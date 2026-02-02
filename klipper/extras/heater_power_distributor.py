import logging
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass

# ---- Constants ----
POLLING_INTERVAL = 1.0
DEFAULT_WATT = 60.0
IDLE_MIN_POWER = 1e-6
TUNING_RESERVATION_BUFFER = 1.05
BACKOFF_BASE_SECONDS = 1.0
MAX_BACKOFF_ATTEMPTS = 5

# Thresholds
AT_HEAT_TOLERANCE = 2.0      # °C below target considered "heating"
EMA_ALPHA = 0.4              # Exponential Moving Average factor for slope

# Built-in PTC Derating Models (Temp °C -> Power Factor 0.0-1.0)
BUILTIN_MODELS = {
    'none':        [(0, 1.0), (400, 1.0)],
    'bambulab':    [(25, 1.00), (100, 0.85), (200, 0.70), (260, 0.62), (320, 0.52)],
    'v1':          [(25, 1.00), (100, 0.82), (200, 0.66), (260, 0.58), (320, 0.48)],
    'rapido_v2':   [(25, 1.00), (100, 0.90), (200, 0.78), (260, 0.70), (320, 0.63)],
    'v3':          [(25, 1.00), (100, 0.88), (200, 0.76), (260, 0.68), (320, 0.60)],
    'triangle70':  [(25, 1.00), (100, 0.80), (200, 0.62), (260, 0.54), (320, 0.45)],
}

def clamp(val, low, high):
    return max(low, min(high, val))

class LinearCurve:
    """Simple linear interpolation for derating curves."""
    def __init__(self, points: List[Tuple[float, float]]):
        self.points = sorted(points, key=lambda p: p[0])
        self.x = [p[0] for p in self.points]
        self.y = [p[1] for p in self.points]

    def get_factor(self, temp_c: float) -> float:
        if temp_c <= self.x[0]: return self.y[0]
        if temp_c >= self.x[-1]: return self.y[-1]
        
        for i in range(len(self.x) - 1):
            if self.x[i] <= temp_c <= self.x[i+1]:
                t = (temp_c - self.x[i]) / (self.x[i+1] - self.x[i])
                return self.y[i] + t * (self.y[i+1] - self.y[i])
        return 1.0

@dataclass
class HeaterContext:
    """Runtime state for a single heater."""
    name: str
    heater: Any
    rated_watt: float
    
    # State tracking
    control_obj: Any = None
    is_mpc: bool = False
    is_tuning: bool = False
    
    # The 'original' max power configured on the heater.
    config_max_power: float = 1.0 
    
    # Snapshot data
    current_temp: float = 0.0
    target_temp: float = 0.0
    slope: float = 0.0
    prev_temp: Optional[float] = None
    prev_time: Optional[float] = None
    
    # Capabilities at current snapshot
    derating_factor: float = 1.0
    available_watts: float = 0.0
    error_count: int = 0
    backoff_until: float = 0.0
    disabled: bool = False
    
    def update_control_type(self):
        """
        Detects control object changes.
        If we switch TO Tuning, we locate the old MPC object and restore its 
        Wattage limit so the calibration math (which reads it later) is correct.
        """
        new_control = self.heater.control
        if id(new_control) == id(self.control_obj):
            return

        old_control = self.control_obj
        old_was_mpc = self.is_mpc
        
        self.control_obj = new_control
        
        # Determine New Type
        ctrl_type = getattr(new_control, 'get_type', lambda: 'unknown')()
        if ctrl_type == 'unknown':
            name = type(new_control).__name__
            if name in ("ControlAutoTune", "MpcCalibrate", "TuningControl"):
                ctrl_type = "tuning"
        
        self.is_mpc = (ctrl_type == 'mpc')
        self.is_tuning = (ctrl_type == 'tuning')

        # ---- CALIBRATION RESTORATION ----
        # If we just switched into Tuning Mode, and we were previously MPC:
        # The 'old_control' is the ControlMPC instance that MpcCalibrate is holding.
        # It is currently clamped to IDLE (1e-6) because we were idle before this.
        # We must restore it to RATED WATTS so the math works.
        if self.is_tuning and old_was_mpc and old_control is not None:
            try:
                # We restore to Rated Watts because that is the standard baseline
                # for calibration math.
                restore_val = self.rated_watt
                
                # If the user had a config limit lower than rated, respect that instead?
                # Usually calibration wants the physical capability. 
                # Let's stick to max(rated, config) to be safe, or just rated.
                # Given PTC, Rated is the safest assumption for "Cold" power.
                
                old_control.heater_max_power = restore_val
                logging.info(f"Distributor: Restored detached MPC '{self.name}' to {restore_val}W for calibration math.")
            except Exception:
                pass

        # Capture Configured Limit (Only if NOT tuning)
        if not self.is_tuning:
            default_pwr = self.rated_watt if self.is_mpc else 1.0
            try:
                val = float(getattr(new_control, 'heater_max_power', default_pwr))
            except (ValueError, AttributeError):
                val = default_pwr
            
            if val <= 0.0: val = default_pwr
            self.config_max_power = val

    def clear_error_backoff(self):
        self.error_count = 0
        self.backoff_until = 0.0

    def update_state(self, eventtime: float, curve: LinearCurve):
        if self.is_tuning: return

        self.current_temp, self.target_temp = self.heater.get_temp(eventtime)
        
        if self.prev_time is not None:
            dt = max(1e-6, eventtime - self.prev_time)
            inst_slope = (self.current_temp - self.prev_temp) / dt
            self.slope = (EMA_ALPHA * inst_slope) + ((1.0 - EMA_ALPHA) * self.slope)
        
        self.prev_temp = self.current_temp
        self.prev_time = eventtime

        self.derating_factor = clamp(curve.get_factor(self.current_temp), 0.0, 1.0)
        self.available_watts = self.rated_watt * self.derating_factor


class HeaterPowerDistributor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        # Configuration
        self.heater_names = config.getlist('heaters')
        self.total_budget_watts = config.getfloat('max_power', above=0.0)
        self.poll_interval = config.getfloat('poll_interval', POLLING_INTERVAL, above=0.0)
        self.priority_boost = config.getfloat('active_extruder_boost', 1.0, above=0.0)
        self.safe_mode = config.getboolean('safe_distribution', True)
        self.sync_mpc_power = config.getboolean('sync_mpc_power', False)

        # Power Model
        model_name = config.getchoice('model', list(BUILTIN_MODELS) + ['custom'], default='none')
        rated_powers = config.getfloatlist('powers', (DEFAULT_WATT,))

        if len(rated_powers) == 1:
            self.rated_watts_map = {h: rated_powers[0] for h in self.heater_names}
        elif len(rated_powers) == len(self.heater_names):
            self.rated_watts_map = {h: p for h, p in zip(self.heater_names, rated_powers)}
        else:
            raise config.error(f"HeaterDistributor: 'powers' length ({len(rated_powers)}) must match heaters ({len(self.heater_names)})")

        if model_name == 'custom':
            pairs = config.getlists('model_points', seps=(',', '\n'), count=2, parser=float)
            points = [(t, f) for t, f in pairs]
            if len(points) < 2:
                raise config.error("HeaterDistributor: Custom model must have at least 2 points.")
        else:
            points = BUILTIN_MODELS[model_name]
        
        self.model_curve = LinearCurve(points)

        # Runtime
        self.heaters: Dict[str, HeaterContext] = {}
        self.pheaters = self.printer.load_object(config, 'heaters')
        
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            'SET_HEATER_DISTRIBUTOR', 'GROUP', self.name,
            self.cmd_SET_HEATER_DISTRIBUTOR,
            desc="Set max total power (watts) for this distributor group"
        )
        self.printer.register_event_handler('klippy:ready', self._handle_ready)

    def _handle_ready(self):
        reactor = self.printer.get_reactor()
        now = reactor.monotonic()
        for name in self.heater_names:
            h_obj = self.pheaters.lookup_heater(name)
            self.heaters[name] = HeaterContext(
                name=name,
                heater=h_obj,
                rated_watt=float(self.rated_watts_map[name])
            )
            self.heaters[name].update_control_type()
            
            if self.safe_mode:
                try:
                    h_obj.control.heater_max_power = IDLE_MIN_POWER
                    # Fix for MPC internal tracking if needed
                    if hasattr(h_obj.control, 'last_power'):
                        h_obj.control.last_power = IDLE_MIN_POWER
                except Exception as exc:
                    self._handle_heater_exception(self.heaters[name], now, exc, "startup_clamp")

        reactor.register_timer(self._update_loop, reactor.monotonic() + self.poll_interval)
        logging.info(f"HeaterDistributor '{self.name}': Budget={self.total_budget_watts}W, Heaters={list(self.heaters.keys())}")

    def cmd_SET_HEATER_DISTRIBUTOR(self, gcmd):
        new_budget = gcmd.get_float('POWER', above=0.0)
        self.total_budget_watts = new_budget
        gcmd.respond_info(f"{self.name}: Budget set to {self.total_budget_watts:.1f} W")

    def _get_active_extruder(self, eventtime):
        th = self.printer.lookup_object('toolhead', None)
        if th:
            try:
                return th.get_status(eventtime).get('extruder')
            except Exception:
                pass
        return None

    def _water_fill(self, demands: Dict[str, float], capacities: Dict[str, float], total_budget: float) -> Dict[str, float]:
        allocations = {n: 0.0 for n in demands}
        remaining_budget = total_budget
        active_set = set(demands.keys())

        while active_set and remaining_budget > 1e-6:
            total_weight = sum(demands[n] for n in active_set)
            if total_weight <= 0: break
            
            share_ratio = remaining_budget / total_weight
            satisfied = set()

            for name in list(active_set):
                requested = demands[name] * share_ratio
                space = capacities[name] - allocations[name]
                add = min(requested, space)
                allocations[name] += add
                if (capacities[name] - allocations[name]) <= 1e-6:
                    satisfied.add(name)
            
            remaining_budget = total_budget - sum(allocations.values())
            active_set -= satisfied
            if not satisfied: break 
        return allocations

    def _restore_default_limits(self, ctx: HeaterContext):
        control = ctx.control_obj or getattr(ctx.heater, 'control', None)
        if control is None:
            return
        try:
            control.heater_max_power = ctx.config_max_power
            if ctx.is_mpc and hasattr(control, 'last_power'):
                control.last_power = ctx.config_max_power
        except Exception:
            logging.exception(f"HeaterDistributor '{self.name}': Failed to restore defaults for '{ctx.name}'")

    def _clamp_pid_integral(self, ctx: HeaterContext, control: Any, limit_val: float):
        try:
            ki = getattr(control, 'Ki', None)
            integ = getattr(control, 'prev_temp_integ', None)
            if ki is None or integ is None or ki <= 0.0:
                return
            integ_max = max(0.0, limit_val) / ki
            if integ < 0.0 or integ > integ_max:
                control.prev_temp_integ = clamp(integ, 0.0, integ_max)
        except Exception as exc:
            logging.debug(
                "HeaterDistributor '%s': Skipping PID integral clamp for '%s': %s",
                self.name, ctx.name, repr(exc)
            )

    def _handle_heater_exception(self, ctx: HeaterContext, eventtime: float, exc: Exception, phase: str):
        if ctx.disabled:
            return

        ctx.error_count += 1
        attempt = ctx.error_count
        backoff_delay = BACKOFF_BASE_SECONDS * (2 ** (attempt - 1))
        disable_now = attempt >= MAX_BACKOFF_ATTEMPTS

        self._restore_default_limits(ctx)

        if disable_now:
            ctx.disabled = True
            ctx.backoff_until = float('inf')
            logging.warning(
                "HeaterDistributor '%s': Disabling control for '%s' after %d/%d failures during %s. "
                "Reverting to configured max power %.4f. Error=%r",
                self.name, ctx.name, attempt, MAX_BACKOFF_ATTEMPTS, phase, ctx.config_max_power, exc
            )
        else:
            ctx.backoff_until = eventtime + backoff_delay
            logging.warning(
                "HeaterDistributor '%s': Exception in '%s' during %s. Backing off %.1fs (attempt %d/%d) and restoring defaults. Error=%r",
                self.name, ctx.name, phase, backoff_delay, attempt, MAX_BACKOFF_ATTEMPTS, exc
            )

    def _apply_power_limits(self, allocations: Dict[str, float], eventtime: float):
        for name, ctx in self.heaters.items():
            if ctx.is_tuning or ctx.disabled or eventtime < ctx.backoff_until:
                continue

            allocated_w = allocations.get(name, 0.0)
            control = ctx.control_obj

            if ctx.is_mpc and self.sync_mpc_power:
                try:
                    if hasattr(control, 'const_heater_power'):
                        control.const_heater_power = ctx.available_watts
                except Exception:
                    pass

            if ctx.target_temp <= 0.0:
                limit_val = IDLE_MIN_POWER if self.safe_mode else ctx.config_max_power
            else:
                if ctx.is_mpc:
                    limit_val = min(allocated_w, ctx.available_watts, ctx.config_max_power)
                else:
                    if ctx.available_watts > 1e-4:
                        limit_val = allocated_w / ctx.available_watts
                    else:
                        limit_val = 0.0
                    limit_val = clamp(limit_val, 0.0, ctx.config_max_power)

            try:
                control.heater_max_power = limit_val
                if ctx.is_mpc and hasattr(control, 'last_power'):
                    if getattr(control, 'last_power') > limit_val:
                        control.last_power = limit_val
                self._clamp_pid_integral(ctx, control, limit_val)
                ctx.clear_error_backoff()
            except Exception as exc:
                self._handle_heater_exception(ctx, eventtime, exc, "apply_power_limits")

    def _update_loop(self, eventtime):
        active_extruder = self._get_active_extruder(eventtime)
        
        demands = {}
        capacities = {}
        reserved_watts = 0.0
        
        needing_heat = []

        for name, ctx in self.heaters.items():
            if ctx.disabled or eventtime < ctx.backoff_until:
                continue

            try:
                ctx.update_control_type()
            except Exception as exc:
                self._handle_heater_exception(ctx, eventtime, exc, "control_detection")
                continue
            
            if ctx.is_tuning:
                reserved_watts += (ctx.rated_watt * TUNING_RESERVATION_BUFFER)
                continue
            try:
                ctx.update_state(eventtime, self.model_curve)
            except Exception as exc:
                self._handle_heater_exception(ctx, eventtime, exc, "state_update")
                continue
            
            if ctx.target_temp > 0:
                is_heating = ctx.current_temp < (ctx.target_temp - AT_HEAT_TOLERANCE)
                
                weight = 1.0
                if name == active_extruder and self.priority_boost > 1.0:
                    weight = self.priority_boost
                if is_heating and ctx.slope < -0.1:
                    weight += 1.0 
                
                demands[name] = weight
                
                if ctx.is_mpc:
                    cap = ctx.available_watts
                else:
                    cap = ctx.available_watts * ctx.config_max_power
                
                capacities[name] = cap
                needing_heat.append(name)

        available_budget = max(0.0, self.total_budget_watts - reserved_watts)

        if not needing_heat:
            self._apply_power_limits({}, eventtime) 
            return eventtime + self.poll_interval

        total_cap_demand = sum(capacities[n] for n in needing_heat)
        
        if total_cap_demand <= available_budget:
            allocations = capacities
        else:
            allocations = self._water_fill(demands, capacities, available_budget)

        self._apply_power_limits(allocations, eventtime)
        return eventtime + self.poll_interval

def load_config_prefix(config):
    return HeaterPowerDistributor(config)
