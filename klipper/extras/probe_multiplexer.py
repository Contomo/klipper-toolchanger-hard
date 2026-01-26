import bisect
import inspect
from dataclasses import dataclass, fields
from typing import Dict, List, Optional, Tuple

try:
    from klippy import probe  # type: ignore
except ImportError:
    from . import probe  # type: ignore


class Sentinel:
    pass


def _probe_accepts_mcu_probe() -> bool:
    """Kalico-style probe.PrinterProbe(config, mcu_probe) vs klipper probe.PrinterProbe(config)."""
    try:
        sig = inspect.signature(probe.PrinterProbe.__init__)
    except Exception:
        return False
    params = list(sig.parameters.values())
    return len(params) >= 3


def _build_homing_helper(config, mcu_probe, probe_offsets, param_helper):
    try:
        sig = inspect.signature(probe.HomingViaProbeHelper.__init__)
        params = list(sig.parameters.values())
        if len(params) >= 5:
            return probe.HomingViaProbeHelper(
                config, mcu_probe, probe_offsets, param_helper
            )
    except Exception:
        pass
    return probe.HomingViaProbeHelper(config, mcu_probe, param_helper)


def _has_probe_helpers() -> bool:
    return all( hasattr(probe, n) for n in (
        "ProbeCommandHelper", "ProbeOffsetsHelper",
        "ProbeParameterHelper", "HomingViaProbeHelper",
        "ProbeSessionHelper",
        )
    )



class _ParamHelperProxy:
    """ A stable object reference for upstream helpers to hold, while allowing
        the active ProbeParameterHelper to be swapped at runtime."""
    def __init__(self, helper):
        self._helper = helper

    def set_helper(self, helper) -> None:
        self._helper = helper

    def get_probe_params(self, gcmd=None):
        return self._helper.get_probe_params(gcmd)

    def __getattr__(self, name: str):
        return getattr(self._helper, name)


class _UpstreamProbeImpl:
    """Upstream probe implementation is composed of helpers."""
    def __init__(self, config, mcu_probe, *, param_helper=None):
        self.printer = config.get_printer()
        self.mcu_probe = mcu_probe
        self.cmd_helper = probe.ProbeCommandHelper(
            config, self, self.mcu_probe.query_endstop
        )
        self.probe_offsets = probe.ProbeOffsetsHelper(config)
        params = inspect.signature(self.probe_offsets.get_offsets).parameters
        self._offsets_accepts_gcmd = (
            ("gcmd" in params)
            or any(p.kind in (p.VAR_POSITIONAL, p.VAR_KEYWORD) for p in params.values())
        )
        self.param_helper = param_helper if param_helper is not None else probe.ProbeParameterHelper(config)
        self.homing_helper = _build_homing_helper(
            config, self.mcu_probe, self.probe_offsets, self.param_helper
        )
        self.probe_session = probe.ProbeSessionHelper(
            config, self.param_helper, self.homing_helper.start_probe_session
        )

    def get_probe_params(self, gcmd=None):
        return self.param_helper.get_probe_params(gcmd)

    def get_offsets(self, gcmd=None):
        if self._offsets_accepts_gcmd:
            return self.probe_offsets.get_offsets(gcmd)
        return self.probe_offsets.get_offsets()

    def get_status(self, eventtime):
        return self.cmd_helper.get_status(eventtime)

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)

    def get_lift_speed(self, gcmd=None):
        params = self.param_helper.get_probe_params(gcmd)
        return params["lift_speed"]


class _FrontEndBase:
    """
    The single global [probe] object registered with the printer.

    - Kalico: a single PrinterProbe instance.
    - Upstream: composition of helper objects.

    This is intentionally separate from per-physical-probe configs (MuxProbe).
    """

    def get_impl(self):
        raise NotImplementedError

    def set_name(self, name: str) -> None:
        raise NotImplementedError

    def set_offsets(self, x: float, y: float, z: float) -> None:
        raise NotImplementedError

    def parse_tuning(self, config):
        """ Parse per-probe tuning parameters from a [probe_multiplexer <name>] section.
            Returns an opaque tuning object understood by apply_tuning()."""
        raise NotImplementedError

    def apply_tuning(self, tuning) -> None:
        raise NotImplementedError

    def clear_tuning(self) -> None:
        return


@dataclass(frozen=True)
class _KalicoTuning:
    speed: float
    lift_speed: float
    sample_count: int
    sample_retract_dist: float
    samples_result: str
    samples_tolerance: float
    samples_retries: int
    drop_first_result: bool
    retry_speed: Optional[float]
    retry_session: Optional[object]

    @classmethod
    def from_config(cls, config, *, supports_retry_speed: bool, supports_retry_session: bool) -> "_KalicoTuning":
        _speed = config.getfloat("speed", 5.0, above=0.0)
        return cls(
            speed=_speed,
            lift_speed=config.getfloat("lift_speed", _speed, above=0.0),
            sample_count=config.getint("samples", 1, minval=1),
            sample_retract_dist=config.getfloat("sample_retract_dist", 2.0, above=0.0),
            samples_result=config.getchoice("samples_result", ["median", "average"], "average"),
            samples_tolerance=config.getfloat("samples_tolerance", 0.100, minval=0.0),
            samples_retries=config.getint("samples_tolerance_retries", 0, minval=0),
            drop_first_result=config.getboolean("drop_first_result", False),
            retry_speed=config.getfloat("retry_speed", _speed, above=0.0) if supports_retry_speed else None,
            retry_session=probe.RetrySession(config) if supports_retry_session else None,
        )


class _FrontEndKalico(_FrontEndBase):
    @classmethod
    def supported(cls) -> bool:
        return _probe_accepts_mcu_probe()

    def __init__(self, config, mcu_probe):
        self._impl = probe.PrinterProbe(config, mcu_probe)

    def get_impl(self):
        return self._impl

    def set_name(self, name: str) -> None:
        self._impl.name = name

    def set_offsets(self, x: float, y: float, z: float) -> None:
        self._impl.x_offset = x
        self._impl.y_offset = y
        self._impl.z_offset = z

    def parse_tuning(self, config):
        return _KalicoTuning.from_config(
            config,
            supports_retry_speed=hasattr(self._impl, "retry_speed"),
            supports_retry_session=hasattr(probe, "RetrySession") and hasattr(self._impl, "retry_session"),
        )

    def apply_tuning(self, tuning: _KalicoTuning) -> None:
        for f in fields(tuning):
            if hasattr(self._impl, f.name):
                setattr(self._impl, f.name, getattr(tuning, f.name))


class _FrontEndUpstream(_FrontEndBase):
    @classmethod
    def supported(cls) -> bool:
        return (not _probe_accepts_mcu_probe()) and _has_probe_helpers()

    def __init__(self, config, mcu_probe):
        base_helper = probe.ProbeParameterHelper(config)
        self._base_helper = base_helper
        self._param_proxy = _ParamHelperProxy(base_helper)
        self._impl = _UpstreamProbeImpl(config, mcu_probe, param_helper=self._param_proxy)

    def get_impl(self):
        return self._impl

    def set_name(self, name: str) -> None:
        self._impl.cmd_helper.name = name

    def set_offsets(self, x: float, y: float, z: float) -> None:
        self._impl.probe_offsets.x_offset = x
        self._impl.probe_offsets.y_offset = y
        self._impl.probe_offsets.z_offset = z

    def parse_tuning(self, config):
        # Upstream provides a parse-only helper, safe to instantiate per probe.
        return probe.ProbeParameterHelper(config)

    def apply_tuning(self, tuning) -> None:
        self._param_proxy.set_helper(tuning)

    def clear_tuning(self) -> None:
        self._param_proxy.set_helper(self._base_helper)


class MuxProbe:
    """One physical probe definition under [probe_multiplexer <name>]."""
    def __init__(self, config, muxer):
        self.printer = config.get_printer()
        self.muxer = muxer
        self.name: str = config.get_name()
        self.number: Optional[int] = config.getint("probe_number", None, minval=0)

        self.mcu_probe = probe.ProbeEndstopWrapper(config)

        self.x_offset: float = config.getfloat("x_offset", 0.0)
        self.y_offset: float = config.getfloat("y_offset", 0.0)
        self.z_offset: float = config.getfloat("z_offset")

        self.tuning = muxer.frontend.parse_tuning(config)

        muxer.add_probe(self)

    def get_offsets(self) -> Tuple[float, float, float]:
        return self.x_offset, self.y_offset, self.z_offset

    def update_offsets(self, *,
        x: Optional[float] = None,
        y: Optional[float] = None,
        z: Optional[float] = None,
    ) -> None:
        for attr, value in (("x_offset", x), ("y_offset", y), ("z_offset", z)):
            if value is not None:
                setattr(self, attr, value)

        if self.muxer.active_probe is self:
            self.muxer.frontend.set_offsets(*self.get_offsets())

    def save_offsets(self) -> None:
        configfile = self.printer.lookup_object("configfile")
        x, y, z = self.get_offsets()
        configfile.set(self.name, "x_offset", "%.3f" % (x,))
        configfile.set(self.name, "y_offset", "%.3f" % (y,))
        configfile.set(self.name, "z_offset", "%.3f" % (z,))

    def get_status(self, eventtime):
        return {
            "name": self.name,
            "number": self.number if self.number is not None else -1,
            "x_offset": self.x_offset,
            "y_offset": self.y_offset,
            "z_offset": self.z_offset,
        }


class ProbeMultiplexer:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        self.toolhead = None

        if not config.fileconfig.has_section(self.name):
            config.fileconfig.add_section(self.name)
        if not config.fileconfig.has_option(self.name, "z_offset"):
            config.fileconfig.set(self.name, "z_offset", "0.0")

        self.probes_by_name: Dict[str, MuxProbe] = {}
        self.probes_by_number: Dict[int, MuxProbe] = {}
        self.probe_names: List[str] = []
        self.probe_numbers: List[int] = []

        self.last_query: Dict[str, int] = {}
        self.active_probe: Optional[MuxProbe] = None
        self.active_name: Optional[str] = None

        self.mcu_probe = EndstopRouter(self.printer)

        if self.printer.lookup_object("probe", default=None):
            raise self.printer.config_error("Cannot have both [probe] and [probe_multiplexer].")

        self.frontend = self._init_frontend(config)

        self.printer.add_object("probe", self.frontend.get_impl())
        self.frontend.set_name(self.name)

        self.printer.register_event_handler("klippy:connect", self._handle_connect)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_ACTIVE_PROBE",
            self.cmd_SET_ACTIVE_PROBE,
            desc=self.cmd_SET_ACTIVE_PROBE_help,
        )
        gcode.register_command(
            "SET_PROBE_OFFSET",
            self.cmd_SET_PROBE_OFFSET,
            desc=self.cmd_SET_PROBE_OFFSET_help,
        )
        gcode.register_command(
            "SAVE_PROBE_OFFSET",
            self.cmd_SAVE_PROBE_OFFSET,
            desc=self.cmd_SAVE_PROBE_OFFSET_help,
        )

    def _init_frontend(self, config) -> _FrontEndBase:
        if _FrontEndKalico.supported():
            return _FrontEndKalico(config, self.mcu_probe)

        if _FrontEndUpstream.supported():
            return _FrontEndUpstream(config, self.mcu_probe)

        if _probe_accepts_mcu_probe():
            raise config.error("Probe module does not support probe_multiplexer.")
        raise config.error("Probe module lacks helpers needed for probe_multiplexer")

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")


    def add_probe(self, mux_probe: MuxProbe):
        key = mux_probe.name.lower()
        if key in self.probes_by_name:
            raise self.printer.config_error("Duplicate probe name: %s" % (mux_probe.name,))
        self.probes_by_name[key] = mux_probe

        pos = bisect.bisect_left(self.probe_names, mux_probe.name)
        self.probe_names.insert(pos, mux_probe.name)

        if mux_probe.number is not None:
            if mux_probe.number in self.probes_by_number:
                raise self.printer.config_error(
                    "Duplicate probe_number: %s" % (mux_probe.number,)
                )
            self.probes_by_number[mux_probe.number] = mux_probe
            posn = bisect.bisect_left(self.probe_numbers, mux_probe.number)
            self.probe_numbers.insert(posn, mux_probe.number)

        self.mcu_probe.add_mcu(mux_probe.mcu_probe)


    def set_active_probe(self, mux_probe: Optional[MuxProbe]):
        if self.active_probe is mux_probe:
            return

        self.active_probe = mux_probe

        if mux_probe is None:
            self.mcu_probe.set_active_mcu(None)
            self.active_name = None
            self.frontend.set_name(self.name)
            self.frontend.clear_tuning()
            return

        self.mcu_probe.set_active_mcu(mux_probe.mcu_probe)
        self.active_name = mux_probe.name

        self.frontend.set_name(mux_probe.name)
        self.frontend.set_offsets(*mux_probe.get_offsets())
        self.frontend.apply_tuning(mux_probe.tuning)


    def _get_probe_from_gcmd(self, gcmd, default=Sentinel) -> Optional[MuxProbe]:
        if default is Sentinel:
            default = self.active_probe
        number = gcmd.get_int("N", gcmd.get_int("NUMBER", None, minval=0), minval=0)
        name = gcmd.get("P", gcmd.get("PROBE", None))

        if number is not None and name is not None:
            raise gcmd.error("Use N/NUMBER or P/PROBE, not both")

        if number is not None:
            if number == -1:
                return None
            p = self.probes_by_number.get(number)
            if p is None:
                raise gcmd.error("Unknown probe_number '%s'" % (number,))
            return p

        if name is not None:
            key = name.strip().lower()
            if key == "none":
                return None
            p = self.probes_by_name.get(key)
            if p is None:
                raise gcmd.error("Unknown probe '%s'" % (name,))
            return p

        return default  # type: ignore


    cmd_SET_ACTIVE_PROBE_help = "Select the active probe by P/PROBE name or N/NUMBER"
    def cmd_SET_ACTIVE_PROBE(self, gcmd):
        probe_sel = self._get_probe_from_gcmd(gcmd)
        self.set_active_probe(probe_sel)
        if probe_sel is None:
            gcmd.respond_info("Active probe cleared")
        else:
            gcmd.respond_info("Active probe set to: %s" % (probe_sel.name,))


    cmd_SET_PROBE_OFFSET_help = "Set offsets for a probe by NUMBER/N or PROBE/P (defaults to active)."
    def cmd_SET_PROBE_OFFSET(self, gcmd):
        probe = self._get_probe_from_gcmd(gcmd)
        if probe is None:
            raise gcmd.error("No probe active and no probe supplied, cannot set offsets.")

        before = probe.get_offsets()
        after = (
            gcmd.get_float("X", before[0]),
            gcmd.get_float("Y", before[1]),
            gcmd.get_float("Z", before[2]),
        )
        if before == after:
            return
        
        probe.update_offsets(x=after[0], y=after[1], z=after[2])
        gcmd.respond_info(
            "Probe '%s' offsets updated: X=%.3f Y=%.3f Z=%.3f"
            % (probe.name, *after)
        )


    cmd_SAVE_PROBE_OFFSET_help = "Save offsets for a probe by NUMBER/N or PROBE/P (defaults to active)."
    def cmd_SAVE_PROBE_OFFSET(self, gcmd):
        probe_sel = self._get_probe_from_gcmd(gcmd)
        if probe_sel is None:
            raise gcmd.error("No probe selected (use SET_ACTIVE_PROBE or pass P/PROBE or N/NUMBER)")
        probe_sel.save_offsets()
        gcmd.respond_info(
            "Probe %s offsets saved to config: X=%.3f Y=%.3f Z=%.3f"
            % (probe_sel.name, *probe_sel.get_offsets())
        )


    def get_status(self, eventtime):
        active_number = self.active_probe.number if self.active_probe else -1
        active_x = self.active_probe.x_offset if self.active_probe else None
        active_y = self.active_probe.y_offset if self.active_probe else None
        active_z = self.active_probe.z_offset if self.active_probe else None
        status = {
            "probe": self.active_name,
            "probe_number": active_number,
            "x_offset": active_x,
            "y_offset": active_y,
            "z_offset": active_z,
            "probe_numbers": list(self.probe_numbers),
            "probe_names": list(self.probe_names),
            "last_probes_query": dict(self.last_query),
        }
        return status


class EndstopRouter:
    """Routes probe/endstop calls to the currently selected MCU probe wrapper."""

    _ROUTED_METHODS = (
        "get_mcu",
        "home_start",
        "home_wait",
        "multi_probe_begin",
        "multi_probe_end",
        "probe_prepare",
        "probe_finish",
    )

    def __init__(self, printer):
        self.printer = printer
        self.active_mcu = None
        self._mcus = []
        self._steppers = []
        self._probing_move_accepts_gcmd: Dict[int, bool] = {}
        self.set_active_mcu(None)

    def add_mcu(self, mcu_probe):
        self._mcus.append(mcu_probe)
        for s in self._steppers:
            mcu_probe.add_stepper(s)

    def add_stepper(self, stepper):
        self._steppers.append(stepper)
        for m in self._mcus:
            m.add_stepper(stepper)

    def get_steppers(self):
        return list(self._steppers)

    def on_error(self, *args, **kwargs):
        raise self.printer.command_error("Cannot interact with probe - no active probe selected.")

    def _require_probe_or_fail(self):
        if not self.active_mcu:
            self.on_error()

    def set_active_mcu(self, mcu_probe):
        self.active_mcu = mcu_probe

        if self.active_mcu is None:
            for name in self._ROUTED_METHODS:
                setattr(self, name, self.on_error)
            return

        for name in self._ROUTED_METHODS:
            fn = getattr(self.active_mcu, name, None)
            if fn is None:
                raise self.printer.command_error(
                    "Selected probe is missing required method '%s'" % (name,)
                )
            setattr(self, name, fn)


    def query_endstop(self, print_time):
        self._require_probe_or_fail()
        return self.active_mcu.query_endstop(print_time)


    def probing_move(self, pos, speed, gcmd=None):
        self._require_probe_or_fail()

        m = getattr(self.active_mcu, "probing_move", None)
        if m is not None:
            mid = id(m)
            accepts = self._probing_move_accepts_gcmd.get(mid)
            if accepts is None:
                accepts = False
                try:
                    sig = inspect.signature(m)
                    params = list(sig.parameters.values())
                    # (pos, speed, gcmd?) - accept 3 positional-ish args
                    accepts = len(params) >= 3
                except Exception:
                    # If introspection fails, try once and cache the result.
                    try:
                        m(pos, speed, gcmd)
                        accepts = True
                    except TypeError:
                        accepts = False
                self._probing_move_accepts_gcmd[mid] = accepts

            if accepts:
                return m(pos, speed, gcmd)
            return m(pos, speed)

        phoming = self.printer.lookup_object("homing")
        return phoming.probing_move(self, pos, speed)

    def get_position_endstop(self):
        if not self.active_mcu:
            return 0.0
        return self.active_mcu.get_position_endstop()



def load_config(config):
    return ProbeMultiplexer(config)


def load_config_prefix(config):
    mux = config.get_printer().load_object(config, "probe_multiplexer")
    return MuxProbe(config, mux)
