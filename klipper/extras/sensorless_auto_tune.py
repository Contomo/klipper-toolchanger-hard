from __future__ import annotations

from typing import List, Dict, Optional, Any, Iterator, Tuple, Deque, Callable
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
import inspect, math

try:
    from klippy import homing  # Kalico
except ImportError:
    from . import homing  # Klipper


TRINAMIC_DRIVERS: List[str] = ["tmc2130", "tmc2208", "tmc2209", "tmc2240", "tmc2660", "tmc5160"]
AXIS_TO_INDEX: Dict[str, int] = {"X": 0, "Y": 1, "Z": 2, "x": 0, "y": 1, "z": 2}

PREFLIGHT_DISTANCE_MAX_MOVE: float = 10.0
MAX_WALL_SHRINK_HOMING: float = 1.0
WALL_REPORT_DELTA: float = 0.1

def _linspace(a: float, b: float, steps: int) -> List[float]:
    if steps <= 1:
        return [0.5 * (a + b)]
    out = []
    for i in range(steps):
        t = i / float(steps - 1)
        out.append(a + (b - a) * t)
    return out


def _cluster_mean(vals: List[float], tol: float, min_samples: int) -> Optional[float]:
    n = len(vals)
    if n < min_samples:
        return None

    s = sorted(vals)
    ps = [0.0]
    for v in s:
        ps.append(ps[-1] + v)

    best_n = 0
    best_mean = None
    i = 0
    for j, vj in enumerate(s):
        while vj - s[i] > tol:
            i += 1
        cnt = j - i + 1
        if cnt < min_samples:
            continue
        mean = (ps[j + 1] - ps[i]) / cnt
        if (cnt > best_n) or (cnt == best_n and (best_mean is None or mean > best_mean)):
            best_n = cnt
            best_mean = mean

    return best_mean


@dataclass(frozen=True)
class HomeResult:
    triggered: bool
    moved_mm: float

W_CUR = 9
W_RUN = 7

RUN_OK = 0
RUN_EARLY = 1
RUN_OVER = 2

@dataclass(frozen=True)
class RunResult:
    status: int
    moved_mm: float = 0.0

@dataclass
class TunePoint:
    sgt: int
    current: float
    baseline: float
    threshold: float
    run_results: List[RunResult] = field(default_factory=list)

    @property
    def early(self) -> int:
        return sum(1 for r in self.run_results if r.status == RUN_EARLY)

    @property
    def no_trigger(self) -> int:
        return sum(1 for r in self.run_results if r.status == RUN_OVER)

    @property
    def moved_wall(self) -> List[float]:
        return [r.moved_mm for r in self.run_results if r.status == RUN_OK]

    @property
    def valid(self) -> bool:
        return self.no_trigger == 0 and self.early == 0 and bool(self.moved_wall)

    def mean_std(self) -> Tuple[float, float]:
        vals = self.moved_wall
        n = len(vals)
        if n == 0:
            return 0.0, 0.0
        mean = sum(vals) / n
        if n < 2:
            return mean, 0.0
        var = sum((x - mean) ** 2 for x in vals) / (n - 1)
        return mean, math.sqrt(var)

    def safety(self) -> float:
        mean, _ = self.mean_std()
        return mean - self.threshold

    def format_row(self, runs_per_test: int) -> str:
        cells = [f"{self.current:>{W_CUR}.2f}"]

        for i in range(runs_per_test):
            if i < len(self.run_results):
                rr = self.run_results[i]
                if rr.status == RUN_OK:
                    cells.append(f"{rr.moved_mm:>{W_RUN}.3f}")
                elif rr.status == RUN_EARLY:
                    cells.append(f"{'X':^{W_RUN}}")
                else:
                    cells.append(f"{'O':^{W_RUN}}")
            else:
                cells.append(f"{'n/a':^{W_RUN}}")

        return "|" + "|".join(cells) + "|"


def format_sgt_table(sgt: int, row: List[TunePoint], runs_per_test: int) -> str:
    lines = [f"SGT {sgt} (X -> early trigger, O -> overtravel, n/a -> skipped)"]

    hdr = r"cur\run"
    header_cells = [f"{hdr:^{W_CUR}}"] + [
        f"{('r' + str(i + 1)):^{W_RUN}}" for i in range(runs_per_test)
    ]
    lines.append("|" + "|".join(header_cells) + "|")

    for pt in row:
        lines.append(pt.format_row(runs_per_test))

    return "\n".join(lines)


@dataclass
class DistanceSeries:
    tol: float
    confirm: int
    window: int
    max_shrink: float
    wall_dist: Optional[float] = None

    vals: Deque[float] = field(default_factory=deque)
    best: Optional[float] = None

    def cap(self) -> Optional[float]:
        return self.best if self.best is not None else self.wall_dist

    def append(self, v: float) -> bool:
        """appends value -> best updated?: True|False"""
        self.vals.append(v)
        if len(self.vals) > self.window:
            self.vals.popleft()

        cand = self._candidate()
        if cand is None:
            return False

        if self.best is None:
            self.best = cand
            return True

        if cand >= self.best:
            self.best = cand
            return True

        if self.best - cand <= self.max_shrink:
            self.best = cand
            return True

        return False

    def _candidate(self) -> Optional[float]:
        if len(self.vals) < self.confirm:
            return None

        s = sorted(self.vals)
        i = 0
        best_cnt = 0
        best_cand = None

        for j, vj in enumerate(s):
            while vj - s[i] > self.tol:
                i += 1
            cnt = j - i + 1
            if cnt < self.confirm:
                continue

            cand = s[j]
            if cnt > best_cnt or (cnt == best_cnt and (best_cand is None or cand > best_cand)):
                best_cnt = cnt
                best_cand = cand

        return best_cand



class SensorlessAutoTune:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")

        self.stepper_name = config.get_name().split(None, 1)[-1]
        if not config.has_section(self.stepper_name):
            raise config.error("Could not find stepper config section '[%s]'" % (self.stepper_name,))

        self.axis = config.get("axis", self.stepper_name[-1])
        if self.axis not in AXIS_TO_INDEX:
            raise config.error("x/y/z only for now.")

        self.tmc_name = None
        for driver in TRINAMIC_DRIVERS:
            sec = "%s %s" % (driver, self.stepper_name)
            if config.has_section(sec):
                self.tmc_name = sec
                break
        if self.tmc_name is None:
            raise config.error("Could not find any TMC driver config section for '%s'" % (self.stepper_name,))
            
        _cur_defaults = self._get_current_defaults(config, self.tmc_name)
        self.home_current_min = config.getfloat(
            "home_current_min", _cur_defaults[0], minval=0.0
        )
        self.home_current_max = config.getfloat(
            "home_current_max", _cur_defaults[1], minval=self.home_current_min
        )
        self.home_current_steps = config.getint(
            "home_current_steps", 1, minval=1
        )

        self.runs_per_test = config.getint("runs_per_test", 1, minval=1)

        self.overtravel_window = config.getfloat("overtravel_window", 0.25, minval=0.0)
        self.restore_speed = config.getfloat("restore_speed", 100.0, above=0.0)

        self.stall_min = config.getint("stall_min", None)
        self.stall_max = config.getint("stall_max", None)

        self._register_mux_axis_ci(
            "SENSORLESS_AUTOTUNE",
            self.cmd_SENSORLESS_AUTOTUNE,
            desc=self.cmd_SENSORLESS_AUTOTUNE_help,
        )

        self._register_mux_axis_ci(
            "SENSORLESS_AUTOTUNE_FIND_CONSTRAINTS",
            self.cmd_SENSORLESS_AUTOTUNE_FIND_CONSTRAINTS,
            desc=self.cmd_SENSORLESS_AUTOTUNE_FIND_CONSTRAINTS_help,
        )

        self.printer.register_event_handler("klippy:connect", self._on_connect)
        self.printer.register_event_handler("homing:homing_move_end", self._on_hmove_end)
        self.printer.register_event_handler("homing:homing_move_begin", self._on_hmove_begin)

        self._hm_kin_spos0: Optional[Dict[str, Any]] = None
        self._hm_last_dvec = None

    def _register_mux_axis_ci(
        self,
        command: str,
        handler: Callable,
        desc: Optional[str] = None,
    ) -> None:
        axis_keys = {self.axis.lower(), self.axis.upper()}
        for key in axis_keys:
            self.gcode.register_mux_command(command, "AXIS", key, handler, desc=desc)

    def _get_current_defaults(self, config, tmc_name):
        tmc_cfg  = config.getsection(tmc_name)
        home_cur = tmc_cfg.getfloat("home_current", None, note_valid=False)
        run_cur  = tmc_cfg.getfloat("run_current",  1.0, note_valid=False)
        if home_cur is not None:
            return (0.80 * home_cur, 1.20 * home_cur)
        return (0.60 * run_cur, run_cur)


    def _on_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        self.kin = self.toolhead.get_kinematics()

        self.tmc_object = self.printer.lookup_object(self.tmc_name)
        self.stall_manager = StallField.from_cmdhelper(self.tmc_object.get_status.__self__, self.toolhead)
        
        self.stall_min = self.config.getint(
            "stall_min",
            self.stall_manager.value_min,
            minval=self.stall_manager.value_min,
            maxval=self.stall_manager.value_max,
        )
        self.stall_max = self.config.getint(
            "stall_max",
            self.stall_manager.value_max,
            minval=self.stall_manager.value_min,
            maxval=self.stall_manager.value_max,
        )


    def _on_hmove_begin(self, hmove, *args):
        self._hm_kin_spos0 = { s.get_name(): s.get_commanded_position() 
                               for s in self.kin.get_steppers() }

    def _on_hmove_end(self, hmove, *args):
        spos0, self._hm_kin_spos0 = self._hm_kin_spos0, None
        if spos0 is None or self._hm_last_dvec is not None:
            return

        trig_steps = {sp.stepper_name: int(sp.trig_pos - sp.start_pos)
                      for sp in hmove.stepper_positions}

        p0 = self.kin.calc_position(spos0)
        p1 = hmove.calc_toolhead_pos(spos0, trig_steps)
        self._hm_last_dvec = [b - a for a, b in zip(p0, p1)]


    def home_once_and_measure(self, max_distance: Optional[float] = None, *, reverse_dir: bool = False) -> HomeResult:
        ax = AXIS_TO_INDEX[self.axis]
        rail = self.kin.rails[ax]
        hi = rail.get_homing_info()

        pos_min, pos_max = rail.get_range()
        if max_distance is None:
            max_distance = 1.5 * (pos_max - pos_min)
        positive_dir_eff = hi.positive_dir ^ reverse_dir

        position_target = pos_max if positive_dir_eff else pos_min

        npos = len(self.toolhead.get_position())
        homepos = [None] * npos
        homepos[ax] = position_target

        start = position_target - (1.0 if positive_dir_eff else -1.0) * max_distance  # type: ignore
        forcepos = [None] * npos
        forcepos[ax] = start

        self._hm_kin_spos0 = None
        self._hm_last_dvec = None

        triggered = True
        with self._preserve_position(self.restore_speed):
            try:
                hs = homing.Homing(self.printer)
                hs.set_axes([ax])
                hs.home_rails([rail], forcepos, homepos)
            except self.printer.command_error as e:
                msg = str(e)
                if "No trigger on" in msg:
                    triggered = False
                elif self._hm_last_dvec is not None:
                    triggered = True
                else:
                    raise

        if (not triggered) or (self._hm_last_dvec is None):
            result = HomeResult(False, 0.0)
        else:
            moved = abs(float(self._hm_last_dvec[ax]))
            result = HomeResult(True, moved if moved > 0.0 else 0.0)

        self._hm_last_dvec = None
        return result


    cmd_SENSORLESS_AUTOTUNE_help = """Run sensorless homing autotune over SGT/current grid."""
    def cmd_SENSORLESS_AUTOTUNE(self, gcmd):
        # ---------------- parameters ----------------
        overtravel = gcmd.get_float(
            "OVERTRAVEL_WINDOW", self.overtravel_window, minval=0.0
        )
        runs_per_test = gcmd.get_int(
            "RUNS_PER_TEST", self.runs_per_test, minval=1
        )
        cur_min = gcmd.get_float(
            "HOME_CURRENT_MIN", self.home_current_min, minval=0.0
        )
        cur_max = gcmd.get_float(
            "HOME_CURRENT_MAX", self.home_current_max, minval=0.0
        )
        cur_steps = gcmd.get_int(
            "HOME_CURRENT_STEPS", self.home_current_steps, minval=1
        )
        stall_min = gcmd.get_int(
            "STALL_MIN", self.stall_min,
            minval=self.stall_manager.value_min,
            maxval=self.stall_manager.value_max,
        )
        stall_max = gcmd.get_int(
            "STALL_MAX", self.stall_max,
            minval=stall_min,
            maxval=self.stall_manager.value_max,
        )

        sgt_vals = self.stall_manager.ordered_least_to_most_sensitive(stall_min, stall_max)
        sgt_vals.reverse()
        currents = _linspace(cur_min, cur_max, cur_steps)

        # ---------------- preflight ----------------
        with self.stall_manager.temporary(self.stall_manager.max_sensitive):
            pre = self.home_once_and_measure(max_distance=PREFLIGHT_DISTANCE_MAX_MOVE)

        if not pre.triggered:
            raise gcmd.error(
                "Precheck failed: no stall within %.1fmm at max sensitivity" % PREFLIGHT_DISTANCE_MAX_MOVE
            )

        min_move_threshold_dist = pre.moved_mm + min(10.0, max(pre.moved_mm * 0.5, 2.0))

        wall_dist: Optional[float] = None
        ax = AXIS_TO_INDEX[self.axis]
        rail = self.kin.rails[ax]
        hi = rail.get_homing_info()

        et = self.printer.get_reactor().monotonic()
        homed_axes = self.toolhead.get_status(et).get("homed_axes", "")
        if self.axis.lower() in homed_axes:
            wall_dist = abs(hi.position_endstop - self.toolhead.get_position()[ax])

        wall = DistanceSeries(
            tol=MAX_WALL_SHRINK_HOMING,
            confirm=2,
            window=8,
            max_shrink=MAX_WALL_SHRINK_HOMING,
            wall_dist=wall_dist,
        )
        gcmd.respond_info(
            "Sensorless autotune starting:\n"
            "  SGT: %d..%d (%d)\n"
            "  Current: %.3f..%.3f (%d)\n"
            "  Runs/pt: %d\n"
            "  Baseline: %.3fmm  Threshold: %.3fmm\n"
            "  Overtravel: %.3fmm"
            % (stall_min, stall_max, len(sgt_vals),
                currents[0], currents[-1], len(currents),
                runs_per_test,
                pre.moved_mm, min_move_threshold_dist,
                overtravel)
        )

        grid: List[List[TunePoint]] = []

        for sgt in sgt_vals:
            row: List[TunePoint] = []
            grid.append(row)

            for cur in currents:
                pt = TunePoint(
                    sgt=sgt,
                    current=cur,
                    baseline=pre.moved_mm,
                    threshold=min_move_threshold_dist,
                )

                with (self._temporary_home_current(cur),
                      self.stall_manager.temporary(int(sgt))):
                    for _ in range(runs_per_test):
                        cap = wall.cap()
                        max_dist = None if cap is None else cap + overtravel
                        r = self.home_once_and_measure(max_distance=max_dist)

                        if not r.triggered:
                            pt.run_results.append(RunResult(RUN_OVER))
                            break

                        cap = wall.cap()
                        wall_slack = MAX_WALL_SHRINK_HOMING + overtravel
                        if r.moved_mm <= min_move_threshold_dist or (cap is not None and r.moved_mm < cap - wall_slack):
                            pt.run_results.append(RunResult(RUN_EARLY))
                            break
                        pt.run_results.append(RunResult(RUN_OK, r.moved_mm))

                        prev_best = wall.best
                        best_changed = wall.append(r.moved_mm)
                        #if (best_changed and
                        #    (prev_best is None or abs(wall.best - prev_best) >= WALL_REPORT_DELTA)   # type: ignore
                        #):
                        #    gcmd.respond_info("new distance to endstop: %.3f" % wall.best)
                row.append(pt)
            gcmd.respond_info(format_sgt_table(sgt, row, runs_per_test))

        candidates = [pt for row in grid for pt in row if pt.valid]

        if not candidates:
            gcmd.respond_info("no stable region found")
            return
                
        candidates.sort(key=lambda p: (p.mean_std()[1], -p.safety()))
        best = candidates[0]
    
        ch = self.tmc_object.get_status.__self__.current_helper
        prev = ch.get_current()

        supports_home_current = (
            hasattr(ch, "set_home_current")
            and hasattr(ch, "req_home_current")
            and len(prev) >= 5
        )
        configfile = self.printer.lookup_object("configfile")

        driver_key = f"driver_{self.stall_manager.field.upper()}"
        configfile.set(self.tmc_name, driver_key, best.sgt)
        lines = [
            f"Staged result(s) for SAVE_CONFIG in [{self.tmc_name}]:",
            f"  {driver_key}: {best.sgt}",
        ]
        if supports_home_current:
            configfile.set(self.tmc_name, "home_current", f"{best.current:.3f}")
            lines.append(f"- home_current: {best.current:.3f}")
        else:
            lines.append(f"- remember to manually set your home current! {best.current:.3f}")

        mean, std = best.mean_std()
        lines += [
            "Best test:",
            f"- mean/std = {mean:.6f} / {std:.3g}",
        ]
        gcmd.respond_info("\n".join(lines))


    cmd_SENSORLESS_AUTOTUNE_FIND_CONSTRAINTS_help = """
    Home both directions from the same start position to estimate span
    """
    def cmd_SENSORLESS_AUTOTUNE_FIND_CONSTRAINTS(self, gcmd):
        margin = gcmd.get_float("MARGIN", 0.1, minval=0.0)

        ax = AXIS_TO_INDEX[self.axis]
        rail = self.kin.rails[ax]
        hi = rail.get_homing_info()

        r1 = self.home_once_and_measure()
        r2 = self.home_once_and_measure(reverse_dir=True)
        if not r2.triggered or not r1.triggered:
            raise gcmd.error("homing seemed to have failed")

        span = r1.moved_mm + r2.moved_mm

        pos_min, pos_max = rail.get_range()
        if abs(hi.position_endstop - pos_min) <= abs(pos_max - hi.position_endstop):
            opposite, stage_key = hi.position_endstop + span - margin, "position_max"
        else:
            opposite, stage_key = hi.position_endstop - span + margin, "position_min"

        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.stepper_name, stage_key, "%.2f" % opposite)
        gcmd.respond_info("Axis span: %.3fmm, new %s: %.2f" % (span, stage_key, opposite))
        gcmd.respond_info("Staged for SAVE_CONFIG in [%s]:" % self.stepper_name)


    @contextmanager
    def _preserve_position(self, restore_speed: float) -> Iterator[None]:
        self.toolhead.wait_moves()

        kin = self.kin
        steppers = list(kin.get_steppers())

        pos0 = list(self.toolhead.get_position())
        limits_entry = list(kin.limits)
        mcu0 = {s.get_name(): int(s.get_mcu_position()) for s in steppers}

        try:
            yield
        finally:
            self.toolhead.wait_moves()

            mcu1 = {s.get_name(): int(s.get_mcu_position()) for s in steppers}

            dstep_mm = {}
            for s in steppers:
                name = s.get_name()
                a = mcu0.get(name)
                b = mcu1.get(name)
                if a is None or b is None:
                    dstep_mm[name] = 0.0
                else:
                    dstep_mm[name] = (b - a) * float(s.get_step_dist())

            dxyz = kin.calc_position(dstep_mm)
            moved_phys = any(abs(float(v)) > 1e-9 for v in dxyz[:3])

            def _restore_limits_to_entry():
                for i, v in enumerate(limits_entry):
                    kin.limits[i] = v

            if not moved_phys:
                self.toolhead.set_position(pos0)
                _restore_limits_to_entry()
                return

            pos1_est = pos0[:]
            for i in range(min(3, len(pos1_est), len(dxyz))):
                pos1_est[i] = float(pos1_est[i]) + float(dxyz[i])

            self.toolhead.set_position(pos1_est)

            limits_tmp = list(kin.limits)
            try:
                for i in range(len(kin.limits)):
                    kin.limits[i] = (float("-inf"), float("inf"))

                coord = [None] * len(pos0)
                for i in range(min(3, len(coord))):
                    coord[i] = float(pos0[i])   # type: ignore

                self.toolhead.manual_move(coord, restore_speed)
                self.toolhead.wait_moves()
            finally:
                for i, v in enumerate(limits_tmp):
                    kin.limits[i] = v

            self.toolhead.set_position(pos0)

            _restore_limits_to_entry()


    @contextmanager
    def _temporary_home_current(self, current: Optional[float]) -> Iterator[None]:
        if current is None:
            yield
            return

        ch = self.tmc_object.get_status.__self__.current_helper
        prev = ch.get_current()
        supports_home_current = (
            len(prev) >= 5 and hasattr(ch, "set_home_current") and hasattr(ch, "req_home_current")
        )
        try:
            if supports_home_current:
                ch.set_home_current(float(current))
            else:
                ch.set_current(float(current), prev[2], self.toolhead.get_last_move_time())
            yield
        finally:
            if supports_home_current:
                ch.set_home_current(prev[4])
            else:
                ch.set_current(prev[0], prev[2], self.toolhead.get_last_move_time())

    def get_status(self, eventtime):
        return {}


class StallField:
    def __init__(
        self,
        *,
        field: str,
        value_min: int,
        value_max: int,
        more_sensitive_step: int,
        fields: object,
        toolhead: object,
        mcu_tmc: object,
    ):
        self.field = field
        self.value_min = int(value_min)
        self.value_max = int(value_max)
        self.more_sensitive_step = int(more_sensitive_step)

        self.toolhead = toolhead

        self._fields = fields
        self._mcu_tmc = mcu_tmc

        reg = self._fields.lookup_register(self.field, None)  # type: ignore
        if reg is None:
            raise RuntimeError("Stall field '%s' has no register" % (self.field,))
        self._reg = reg

        sig = None
        try:
            sig = inspect.signature(self._mcu_tmc.set_register)  # type: ignore
        except Exception:
            pass
        self._supports_print_time = bool(sig is None or len(sig.parameters) >= 3)

    @classmethod
    def from_cmdhelper(cls, cmdhelper: object, toolhead) -> "StallField":
        fields = cmdhelper.fields  # type: ignore
        for field in ("sgthrs", "sg4_thrs", "sgt"):
            reg = fields.lookup_register(field, None)
            if reg not in fields.all_fields or field not in fields.all_fields[reg]:
                continue

            fmask = int(fields.all_fields[reg][field])
            fwidth = int(fmask.bit_count())

            if field in getattr(fields, "signed_fields", ()):
                value_min = -(1 << (fwidth - 1))
                value_max = (1 << (fwidth - 1)) - 1
                more_sensitive_step = -1
            else:
                value_min = 0
                value_max = (1 << fwidth) - 1
                more_sensitive_step = +1

            return cls(
                field=field,
                value_min=value_min,
                value_max=value_max,
                more_sensitive_step=more_sensitive_step,
                fields=fields,
                toolhead=toolhead,
                mcu_tmc=cmdhelper.mcu_tmc,  # type: ignore
            )
        raise RuntimeError("Unable to detect a StallGuard threshold field on this TMC driver")

    def clamp(self, v: int) -> int:
        lo = min(self.value_min, self.value_max)
        hi = max(self.value_min, self.value_max)
        v = int(v)
        if v < lo:
            return lo
        if v > hi:
            return hi
        return v

    def sensitivity_key(self, v: int) -> int:
        return int(v) * self.more_sensitive_step

    @property
    def max_sensitive(self) -> int:
        return self.value_max if self.more_sensitive_step > 0 else self.value_min

    @property
    def min_sensitive(self) -> int:
        return self.value_min if self.more_sensitive_step > 0 else self.value_max

    def ordered_least_to_most_sensitive(self, vmin: int, vmax: int) -> List[int]:
        vmin, vmax  = self.clamp(vmin), self.clamp(vmax)
        lo, hi = min(vmin, vmax), max(vmin, vmax)
        vals = list(range(int(lo), int(hi) + 1))
        vals.sort(key=self.sensitivity_key)
        return vals

    def read(self) -> int:
        return int(self._fields.get_field(self.field))  # type: ignore

    def write(self, value: int):
        v = self.clamp(value)
        reg_val = self._fields.set_field(self.field, v)  # type: ignore
        if self._supports_print_time:
            self._mcu_tmc.set_register(self._reg, reg_val, self.toolhead.get_last_move_time())  # type: ignore
        else:
            self._mcu_tmc.set_register(self._reg, reg_val)  # type: ignore

    @contextmanager
    def temporary(self, value: int) -> Iterator[None]:
        prev = self.read()
        try:
            self.write(value)
            yield
        finally:
            self.write(prev)


def load_config_prefix(config):
    return SensorlessAutoTune(config)
