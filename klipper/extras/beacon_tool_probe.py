import importlib
import logging
from . import probe

try:
    beacon_mod = importlib.import_module("beacon")
except ImportError:
    beacon_mod = None


class _BeaconConfigProxy:
    def __init__(self, base_config, beacon_section_name):
        self._base = base_config
        self._name = beacon_section_name

    def get_name(self):
        return self._name

    def __getattr__(self, item):
        return getattr(self._base, item)


class BeaconMCUAdapter:
    def __init__(self, base):
        self.base = base
        self.get_mcu = base.get_mcu
        self.add_stepper = base.add_stepper
        self.get_steppers = base.get_steppers
        self.home_start = base.home_start
        self.home_wait = base.home_wait
        self.query_endstop = base.query_endstop
        self.get_position_endstop = base.get_position_endstop

    def multi_probe_begin(self, *args, **kwargs):
        # No special handling needed for Beacon endstops
        return

    def multi_probe_end(self, *args, **kwargs):
        return

    def probe_prepare(self, hmove):
        return

    def probe_finish(self, hmove):
        return


class BeaconProbeSessionHelper:
    def __init__(self, config, beacon_probe, mode):
        self.printer = config.get_printer()
        self.beacon = beacon_probe
        self.mode = mode
        gcode = self.printer.lookup_object("gcode")
        self.dummy_gcode_cmd = gcode.create_gcode_command("", "", {})
        self.speed = config.getfloat("speed", 5.0, above=0.0)
        self.lift_speed = config.getfloat("lift_speed", self.speed, above=0.0)
        self.sample_count = config.getint("samples", 1, minval=1)
        self.sample_retract_dist = config.getfloat(
            "sample_retract_dist", 2.0, above=0.0
        )
        atypes = {"median": "median", "average": "average"}
        self.samples_result = config.getchoice("samples_result", atypes, "average")
        self.samples_tolerance = config.getfloat(
            "samples_tolerance", 0.100, minval=0.0
        )
        self.samples_retries = config.getint("samples_tolerance_retries", 0, minval=0)
        self.multi_probe_pending = False
        self.results = []
        self.printer.register_event_handler(
            "gcode:command_error", self._handle_command_error
        )

    def _handle_command_error(self):
        if self.multi_probe_pending:
            try:
                self.end_probe_session()
            except Exception:
                logging.exception("Multi-probe end")

    def _probe_state_error(self):
        raise self.printer.command_error(
            "Internal probe error - start/end probe session mismatch"
        )

    def get_probe_params(self, gcmd=None):
        if gcmd is None:
            gcmd = self.dummy_gcode_cmd
        probe_speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        samples = gcmd.get_int("SAMPLES", self.sample_count, minval=1)
        sample_retract_dist = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        samples_tolerance = gcmd.get_float(
            "SAMPLES_TOLERANCE", self.samples_tolerance, minval=0.0
        )
        samples_retries = gcmd.get_int(
            "SAMPLES_TOLERANCE_RETRIES", self.samples_retries, minval=0
        )
        samples_result = gcmd.get("SAMPLES_RESULT", self.samples_result)
        return {
            "probe_speed": probe_speed,
            "lift_speed": lift_speed,
            "samples": samples,
            "sample_retract_dist": sample_retract_dist,
            "samples_tolerance": samples_tolerance,
            "samples_tolerance_retries": samples_retries,
            "samples_result": samples_result,
        }

    def _inject_mode(self, gcmd):
        if gcmd is None:
            return self.dummy_gcode_cmd
        params = dict(gcmd.get_command_parameters())
        params["PROBE_METHOD"] = self.mode
        gcode = self.printer.lookup_object("gcode")
        return gcode.create_gcode_command(
            gcmd.get_command(), gcmd.get_commandline(), params
        )

    def start_probe_session(self, gcmd):
        if self.multi_probe_pending:
            self._probe_state_error()
        self.beacon.multi_probe_begin()
        self.multi_probe_pending = True
        self.results = []
        return self

    def end_probe_session(self):
        if not self.multi_probe_pending:
            self._probe_state_error()
        self.results = []
        self.multi_probe_pending = False
        self.beacon.multi_probe_end()

    def run_probe(self, gcmd):
        if not self.multi_probe_pending:
            self._probe_state_error()
        params = self.get_probe_params(gcmd)
        toolhead = self.printer.lookup_object("toolhead")
        probexy = toolhead.get_position()[:2]
        retries = 0
        positions = []
        sample_count = params["samples"]
        mode_gcmd = self._inject_mode(gcmd)
        while len(positions) < sample_count:
            pos = self.beacon.run_probe(mode_gcmd)
            positions.append(pos)
            z_positions = [p[2] for p in positions]
            if max(z_positions) - min(z_positions) > params["samples_tolerance"]:
                if retries >= params["samples_tolerance_retries"]:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                positions = []
            if len(positions) < sample_count:
                toolhead.manual_move(
                    probexy + [pos[2] + params["sample_retract_dist"]],
                    params["lift_speed"],
                )
        epos = probe.calc_probe_z_average(positions, params["samples_result"])
        self.results.append(epos)

    def pull_probed_results(self):
        res = self.results
        self.results = []
        return res


class BeaconToolProbe:
    def __init__(self, config):
        self.tool = config.getint("tool")
        self.printer = config.get_printer()
        self.name = config.get_name()
        sec_parts = self.name.split()
        default_sensor_name = sec_parts[1] if len(sec_parts) > 1 else None
        self.sensor_name = config.get("sensor", default_sensor_name)
        self.mode = config.getchoice(
            "mode", {"proximity": "proximity", "contact": "contact"}, "proximity"
        )
        self.beacon = self._ensure_beacon_sensor(config)
        base_mcu = (
            self.beacon.mcu_probe
            if self.mode == "proximity"
            else self.beacon.mcu_contact_probe
        )
        self.mcu_probe = BeaconMCUAdapter(base_mcu)
        self.probe_session = BeaconProbeSessionHelper(config, self.beacon, self.mode)
        self.endstop = self.printer.load_object(config, "tool_probe_endstop")
        self.endstop.add_probe(config, self)

    def _ensure_beacon_sensor(self, config):
        beacons = self.printer.lookup_object("beacons", None)
        key = None if self.sensor_name in (None, "") else self.sensor_name
        if beacons is None or key not in beacons.sensors:
            if beacon_mod is None:
                raise config.error(
                    "Beacon module not available; install/enable beacon first"
                )
            proxy_name = "beacon sensor %s" % (self.sensor_name or "")
            proxy_cfg = _BeaconConfigProxy(config, proxy_name)
            beacon_mod.load_config_prefix(proxy_cfg)
            beacons = self.printer.lookup_object("beacons", None)
        if beacons is None or key not in beacons.sensors:
            raise config.error(
                "Beacon sensor '%s' not found; ensure it is configured"
                % (key,)
            )
        return beacons.sensors[key]

    def get_probe_params(self, gcmd=None):
        return self.probe_session.get_probe_params(gcmd)

    def get_offsets(self):
        return self.beacon.get_offsets()

    def start_probe_session(self, gcmd):
        return self.probe_session.start_probe_session(gcmd)


def load_config_prefix(config):
    return BeaconToolProbe(config)
