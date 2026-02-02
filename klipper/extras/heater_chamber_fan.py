import logging

from . import fan
from .heaters import ControlPID, ControlBangBang

PIN_MIN_TIME = getattr(fan, 'FAN_MIN_TIME', 0.100)


class _FanConfigProxy:
    """Thin shim for fan.Fan: only map fan_pin -> pin."""
    def __init__(self, base_cfg):
        self._c = base_cfg

    def get_printer(self):
        return self._c.get_printer()

    def get_name(self):
        return self._c.get_name()

    def get(self, opt, default=None, note_valid=True):
        if opt == 'pin':
            return self._c.get('fan_pin')
        return self._c.get(opt, default, note_valid=note_valid)

    def __getattr__(self, name):
        return getattr(self._c, name)


class _HeaterShim:
    """Adapter so standard Control* classes can drive the fan."""
    def __init__(self, parent, max_power, smooth_time):
        self._parent = parent
        self._max_power = max_power
        self._smooth_time = smooth_time

    def get_max_power(self):
        return self._max_power

    def get_smooth_time(self):
        return self._smooth_time

    def set_pwm(self, read_time, value):
        self._parent._apply_output(value, read_time)

    # kalico compat
    def set_inv_smooth_time(self, inv):
        if inv and inv > 0.0:
            self._parent.inv_smooth_time = inv
            self._smooth_time = 1.0 / inv

    def get_temp(self, eventtime):
        return self._parent.get_temp(eventtime)

    @property
    def reactor(self):
        return self._parent.printer.get_reactor()

class PrinterHeaterChamberFan:
    """Temperature‑controlled chamber fan with optional post‑print linger."""

    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        self.name = config.get_name()
        self.short_name = self.name.split()[-1]

        # Sensor registration
        self.pheaters = self.printer.load_object(config, 'heaters')
        self.sensor = self.pheaters.setup_sensor(config)
        gcode_id = config.get('gcode_id', default='C')
        self.pheaters.register_sensor(config, self, gcode_id=gcode_id)

        # Bounds mirror heater semantics
        self.min_temp = config.getfloat('min_temp', minval=-273.15)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self._sensor_cb)

        # Input smoothing for control loop stability
        self.smooth_time = config.getfloat('smooth_time', 1.0, above=0.0)
        self.inv_smooth_time = 1.0 / self.smooth_time
        self.last_temp = 0.0
        self.smoothed_temp = 0.0
        self.last_temp_time = 0.0

        # Control algorithm choice
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        self.control_mode = config.getchoice('control', algos, default='pid')

        # Hardware fan instance
        self.fan = fan.Fan(_FanConfigProxy(config), default_shutdown_speed=0.0) # fan fan fan fan

        # Power limits and on/off semantics
        self.min_power = config.getfloat('min_power', 0.0, minval=0.0, maxval=1.0)
        self.max_power = config.getfloat('max_power', 1.0, minval=0.0, maxval=1.0)
        self.off_below = config.getfloat('off_below', 0.0, minval=0.0, maxval=1.0)
        if self.min_power > self.max_power:
            raise config.error("Invalid config: min_power (%.3f) must be <= max_power (%.3f)" % (self.min_power, self.max_power))
        if self.min_power < self.off_below:
            raise config.error("Invalid config: min_power (%.3f) must be >= off_below (%.3f)" % (self.min_power, self.off_below))

        # State
        self.last_fan_speed = 0.0
        self.target_temp = config.getfloat('target_temp', 0.0)
        self.state = 'idle'  # 'idle' | 'active' | 'linger'
        self.manual_override = False

        # Linger behavior after printing
        self.auto_linger = config.getboolean('auto_linger', True)
        self.linger_seconds_default = config.getfloat('linger_time', 0.0, minval=0.0)
        self.coast_power_default = config.getfloat('linger_power', self.min_power, minval=0.0, maxval=1.0)
        self.linger_end_time = 0.0
        self.linger_power = self.coast_power_default

        # Optional gating
        self.gate_against = config.get('gate_against', default='heater_bed', note_valid=False)
        self.gate_mode = config.getchoice('gate_mode', {'target': 'target', 'current': 'current', 'either': 'either', None: None}, default='target')

        # Control loop wiring
        self._last_printing = False
        self._heater_shim = _HeaterShim(self, max_power=self.max_power, smooth_time=self.smooth_time)
        try: # klipper
            self.controller = self.control_mode(self._heater_shim, config)

        except AttributeError: # Kalico signature: (profile, heater, load_clean)
            if self.control_mode is ControlBangBang:
                _profile = {
                    'control': 'watermark',
                    'max_delta': config.getfloat('max_delta', 2.0),
                    'smooth_time': None,
                }
            else:
                _profile = {
                    'control': 'pid',
                    'pid_kp': config.getfloat('pid_kp'),
                    'pid_ki': config.getfloat('pid_ki'),
                    'pid_kd': config.getfloat('pid_kd'),
                    'smooth_time': None,
                }
            self.controller = self.control_mode(_profile, self._heater_shim, True)
        self.timer_period = 0.2
        self._timer = None

        # Startup hook
        self.printer.register_event_handler('klippy:ready', self._on_ready)

        # G-code interface
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('SET_HEATER_TEMPERATURE', 'HEATER', self.short_name,
                                   self.cmd_SET_HEATER_TEMPERATURE,
                                   desc='Sets a heater-chamber target temperature')
        gcode.register_mux_command('CHAMBER_FAN_LINGER', 'HEATER', self.short_name,
                                   self.cmd_CHAMBER_FAN_LINGER,
                                   desc='Run chamber fan at coast power for SECONDS (starts/restarts linger)')
        gcode.register_mux_command('SET_FAN_SPEED', 'FAN', self.short_name,
                                   self.cmd_SET_FAN_SPEED,
                                   desc='Manual override: set chamber fan duty (0..1), cancels temperature control')

    # ---- Lifecycle / control loop -------------------------------------------------
    def _on_ready(self):
        if self.gate_mode is not None:
            try:
                self.pheaters.lookup_heater(self.gate_against)
            except Exception:
                raise self.config.error(f"gate_against '{self.gate_against}' not found")
        r = self.printer.get_reactor()
        self._timer = r.register_timer(self._callback, r.monotonic() + PIN_MIN_TIME)

    def _sensor_cb(self, read_time, temp):
        """Simple leaky-integrator smoothing"""
        time_diff = read_time - self.last_temp_time
        self.last_temp = temp
        self.last_temp_time = read_time
        temp_diff = temp - self.smoothed_temp
        adj = min(max(time_diff * self.inv_smooth_time, 0.0), 1.0)
        self.smoothed_temp += temp_diff * adj

    def _heater_now(self, heater_name):
        if self.gate_mode is not None:
            h = self.pheaters.lookup_heater(heater_name)
            eventtime = self.printer.get_reactor().monotonic()
            return h.get_temp(eventtime)
        return 0.0, 0.0

    def _gating_allows_wait(self, target):
        if not target or target <= 0.0:
            return True
        cur, tgt = self._heater_now(self.gate_against)
        if self.gate_mode == 'target':
            return tgt >= target
        if self.gate_mode == 'current':
            return cur >= target
        return tgt >= target or cur >= target

    def _reset_control_state(self):
        """Reset PID state when (re)activating the controller."""
        if not hasattr(self, 'controller'):
            return
        now = self.printer.get_reactor().monotonic()
        # Prefer the most recent raw reading; fall back to smoothed value
        current_temp = self.last_temp or self.smoothed_temp
        if not current_temp and self.target_temp:
            current_temp = self.target_temp
        self.last_temp = current_temp
        self.smoothed_temp = current_temp
        self.last_temp_time = now

        ctrl = self.controller
        if hasattr(ctrl, 'prev_temp'):
            ctrl.prev_temp = current_temp
        if hasattr(ctrl, 'prev_temp_time'):
            ctrl.prev_temp_time = now
        if hasattr(ctrl, 'prev_temp_deriv'):
            ctrl.prev_temp_deriv = 0.0
        if hasattr(ctrl, 'prev_temp_integ'):
            ctrl.prev_temp_integ = 0.0

    def _apply_output(self, controller_out, eventtime=None):
        """Enforce limits and semantics"""
        duty = max(0.0, min(self.max_power, float(controller_out)))
        if not self.manual_override and self.state == 'active' and self.target_temp > 0.0:
            if duty < self.min_power:
                duty = self.min_power
        if 0.0 < duty < self.off_below:
            duty = 0.0
        if duty != self.last_fan_speed:
            self.last_fan_speed = duty
            if eventtime is None:
                eventtime = self.printer.get_reactor().monotonic()
            # Convert reactor time to the MCU print_time domain before issuing PWM
            print_time = self.fan.get_mcu().estimated_print_time(eventtime)
            self.fan.set_speed(print_time + PIN_MIN_TIME, duty)

    def _callback(self, eventtime):
        try:
            ps = self.printer.lookup_object('print_stats', default=None)
            if ps is not None:
                printing_now = (str(ps.state).lower() == 'printing')
                if self._last_printing and not printing_now and self.auto_linger:
                    if self.linger_seconds_default > 0.0:
                        self._start_linger(self.linger_seconds_default, self.coast_power_default, eventtime)
                self._last_printing = printing_now

            if self.manual_override:
                self.state = 'idle' if self.last_fan_speed <= 0.0 else 'active'
                return eventtime + self.timer_period

            if self.state == 'linger':
                if eventtime >= self.linger_end_time:
                    self.state = 'idle'
                    self.target_temp = 0.0
                    self._apply_output(0.0, eventtime)
                else:
                    self._apply_output(self.linger_power, eventtime)
                return eventtime + self.timer_period

            if self.target_temp > 0.0:
                self.state = 'active'
                self.controller.temperature_update(eventtime, self.smoothed_temp, self.target_temp)
            else:
                self.state = 'idle'
                self._apply_output(0.0, eventtime)

            return eventtime + self.timer_period

        except Exception:
            # Fail safe: stop driving and stop the timer
            logging.exception("heater_chamber_fan %s callback error", self.short_name)
            try:
                self._apply_output(0.0, eventtime)
            except Exception:
                pass
            return self.printer.get_reactor().NEVER

    # ---- Public heater-like interface --------------------------------------------
    def get_status(self, eventtime):
        linger_left = max(0.0, self.linger_end_time - eventtime)
        fan_status = self.fan.get_status(eventtime)
        fan_value = float(fan_status.get('value', self.last_fan_speed))
        fan_power = float(fan_status.get('power', self.last_fan_speed))
        return {
            'temperature': round(self.smoothed_temp, 2),
            'target': float(self.target_temp),
            'power': fan_power,
            'value': fan_value,
            'fan_speed': fan_value,
            'state': self.state,
            'linger_time_left': round(linger_left, 2) if self.state == 'linger' else 0.0,
        }

    def get_temp(self, eventtime):
        return (self.smoothed_temp, self.target_temp)

    # ---- G-code commands ----------------------------------------------------------
    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.0)
        wait = gcmd.get_int('WAIT', 0, minval=0, maxval=1)
        previous_target = self.target_temp
        if temp and (temp < self.min_temp or temp > self.max_temp):
            raise gcmd.error("Requested temperature (%.1f) out of range (%.1f:%.1f)" % (temp, self.min_temp, self.max_temp))
        self.manual_override = False
        self.target_temp = float(temp)

        r = self.printer.get_reactor()
        if self._timer:
            r.update_timer(self._timer, r.monotonic() + PIN_MIN_TIME)
        self.linger_end_time = 0.0
        self.state = 'active' if self.target_temp > 0.0 else 'idle'

        if self.target_temp > 0.0 and previous_target <= 0.0:
            self._reset_control_state()

        if wait and self.target_temp > 0.0:
            if not self._gating_allows_wait(self.target_temp):
                gcmd.respond_info("CHAMBER_FAN %s: skipping WAIT (gate=%s against %s not satisfied)" % (self.short_name, self.gate_mode, self.gate_against))
                return
            reactor = self.printer.get_reactor()
            et = reactor.monotonic()
            while not self.printer.is_shutdown():
                if self.smoothed_temp >= self.target_temp - 0.5:
                    break
                gcmd.respond_raw(self._format_m105_line(et))
                et = reactor.pause(et + 1.0)

    def _format_m105_line(self, eventtime):
        return "%s:%.1f /%.1f" % (self.short_name, self.smoothed_temp, self.target_temp)

    def _start_linger(self, seconds, power, now=None):
        if now is None:
            now = self.printer.get_reactor().monotonic()
        self.linger_end_time = now + max(0.0, float(seconds))
        self.linger_power = max(0.0, min(1.0, float(power)))
        self.state = 'linger'
        self.target_temp = 0.0
        self.manual_override = False

    def cmd_CHAMBER_FAN_LINGER(self, gcmd):
        seconds = gcmd.get_float('SECONDS', self.linger_seconds_default, minval=0.0)
        power = gcmd.get_float('POWER', self.coast_power_default, minval=0.0, maxval=1.0)
        self._start_linger(seconds, power)

    def cmd_SET_FAN_SPEED(self, gcmd):
        """Manual override; disable temperature control until changed again"""
        speed = gcmd.get_float('SPEED', None, 0.0)
        template = gcmd.get('TEMPLATE', None)
        if (speed is None) == (template is None):
            raise gcmd.error('SET_FAN_SPEED must specify SPEED or TEMPLATE')
        if template is not None:
            raise gcmd.error('SET_FAN_SPEED TEMPLATE is not supported on heater_chamber_fan')
        self.target_temp = 0.0
        self.manual_override = True
        self._apply_output(speed, self.printer.get_reactor().monotonic())


def load_config_prefix(config):
    return PrinterHeaterChamberFan(config)
