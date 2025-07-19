# Per-tool Z-Probe support
#
# Copyright (C) 2023 Viesturs Zarins <viesturz@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import probe

if not all(hasattr(probe, attr) for attr in (
    'ProbeParameterHelper',
    'ProbeSessionHelper',
    'HomingViaProbeHelper',
    'ProbeCommandHelper')):
    try:
        from .kalico_compat import ensure_probe_backports
    except ImportError:
        pass
    else:
        ensure_probe_backports(probe)

# Virtual endstop, using a tool attached Z probe in a toolchanger setup.
# Tool endstop change may be done either via SET_ACTIVE_TOOL_PROBE TOOL=99
# Or via auto-detection of single open tool probe via DETECT_ACTIVE_TOOL_PROBE
class ToolProbeEndstop:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name()
        self.tool_probes = {}
        self.last_query = {} # map from tool number to endstop state
        self.active_probe = None
        self.active_tool_number = -1
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.crash_detection_active = False
        self.crash_lasttime = 0.
        self.mcu_probe = EndstopRouter(self.printer)
        self.param_helper = probe.ProbeParameterHelper(config)
        self.homing_helper = probe.HomingViaProbeHelper(config, self.mcu_probe, self.param_helper)
        self.probe_session = probe.ProbeSessionHelper(config, self.param_helper, self.homing_helper.start_probe_session)
        self.cmd_helper = probe.ProbeCommandHelper(config, self, self.mcu_probe.query_endstop)
        self._active_session = None

        # Emulate the probe object, since others rely on this.
        if self.printer.lookup_object('probe', default=None):
            raise self.printer.config_error('Cannot have both [probe] and [tool_probe_endstop].')
        self.printer.add_object('probe', self)

        self.crash_mintime = config.getfloat('crash_mintime', 0.5, above=0.)
        self.crash_gcode = self.gcode_macro.load_template(config, 'crash_gcode', '')
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        # Register PROBE/QUERY_PROBE commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('SET_ACTIVE_TOOL_PROBE', self.cmd_SET_ACTIVE_TOOL_PROBE,
                                    desc=self.cmd_SET_ACTIVE_TOOL_PROBE_help)
        self.gcode.register_command('DETECT_ACTIVE_TOOL_PROBE', self.cmd_DETECT_ACTIVE_TOOL_PROBE,
                                    desc=self.cmd_DETECT_ACTIVE_TOOL_PROBE_help)
        self.gcode.register_command('START_TOOL_PROBE_CRASH_DETECTION', self.cmd_START_TOOL_PROBE_CRASH_DETECTION,
                                    desc=self.cmd_START_TOOL_PROBE_CRASH_DETECTION_help)
        self.gcode.register_command('STOP_TOOL_PROBE_CRASH_DETECTION', self.cmd_STOP_TOOL_PROBE_CRASH_DETECTION,
                                    desc=self.cmd_STOP_TOOL_PROBE_CRASH_DETECTION_help)

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self._detect_active_tool()

    def get_offsets(self):
        if self.active_probe:
            return self.active_probe.get_offsets()
        return 0.0, 0.0, 0.0
    
    def get_probe_params(self, gcmd=None):
        if self.active_probe:
            return self.active_probe.get_probe_params(gcmd)
        raise self.printer.command_error("No active tool probe")
    
    def get_lift_speed(self, gcmd=None):
        if self.active_probe:
            return self.active_probe.get_probe_params(gcmd)['lift_speed']
        return self.param_helper.get_probe_params(gcmd)['lift_speed']

    def start_probe_session(self, gcmd):
        if self.active_probe:
            return self.active_probe.start_probe_session(gcmd)
        raise self.printer.command_error("No active tool probe")

    def _ensure_active_probe(self, gcmd=None):
        if self.active_probe:
            return
        if gcmd is None:
            gcode = self.printer.lookup_object('gcode')
            gcmd = gcode.create_gcode_command("", "", {})
        self._ensure_active_tool_or_fail(gcmd)

    def multi_probe_begin(self, *args, **kwargs):
        self._ensure_active_probe()
        self._active_session = None

    def _get_session(self, gcmd):
        self._ensure_active_probe(gcmd)
        if self._active_session is None:
            self._active_session = self.active_probe.start_probe_session(gcmd)
        return self._active_session

    def run_probe(self, gcmd):
        session = self._get_session(gcmd)
        session.run_probe(gcmd)
        results = session.pull_probed_results()
        if not results:
            raise self.printer.command_error("Probe did not report a result")
        return results[-1]

    def multi_probe_end(self):
        if self._active_session is not None:
            try:
                self._active_session.end_probe_session()
            finally:
                self._active_session = None

    def add_probe(self, config, tool_probe):
        if (tool_probe.tool in self.tool_probes):
            raise config.error("Duplicate tool probe nr: %s" % (tool_probe.tool,))
        self.tool_probes[tool_probe.tool] = tool_probe
        self.mcu_probe.add_mcu(tool_probe.mcu_probe)

    def set_active_probe(self, tool_probe):
        if self.active_probe == tool_probe:
            return
        self.active_probe = tool_probe
        if self.active_probe:
            self.mcu_probe.set_active_mcu(tool_probe.mcu_probe)
            self.active_tool_number = self.active_probe.tool
            self.cmd_helper.name = self.active_probe.name
        else:
            self.mcu_probe.set_active_mcu(None)
            self.active_tool_number = -1
            self.cmd_helper.name = self.name
        self._active_session = None

    def _query_open_tools(self, tool_number=None):
        if tool_number is not None and tool_number not in self.tool_probes:
            tool_number = None
        poll_s = 0.005
        prev = dict(self.last_query)
        self.last_query.clear()
        deadline_pt = self.toolhead.get_last_move_time()

        def _query_probes(now):
            candidates = []
            for tool_probe in self.tool_probes.values():
                mcu_probe = tool_probe.mcu_probe
                mcu = mcu_probe.get_mcu()
                endstop = getattr(mcu_probe, "mcu_endstop", None)
                if (getattr(mcu, "non_critical_disconnected", False)
                    or endstop is None
                    or getattr(endstop, "_query_cmd", None) is None):
                    continue
                mcu_now = mcu.estimated_print_time(now)
                triggered = mcu_probe.query_endstop(mcu_now)
                self.last_query[tool_probe.tool] = triggered
                if not triggered and tool_probe.tool >= 0:
                    candidates.append(tool_probe)
            return candidates
        
        while True:
            host_now = self.reactor.monotonic()
            # default exit if we didnt detect one
            if self.toolhead.mcu.estimated_print_time(host_now) >= deadline_pt:
                return _query_probes(host_now)
            keys = ([tool_number] if tool_number is not None
                    else [tp.tool for tp in self.tool_probes.values()])
            # really early exit if its already detected right now
            candidates = _query_probes(host_now)
            if tool_number is not None and self.last_query.get(tool_number, 1) == 0:
                return candidates
            # Early-exit on falling edge: triggered (True) -> open (False)
            if any(bool(prev.get(k, self.last_query.get(k, 0))) and not bool(self.last_query.get(k, 0))
                   for k in keys):
                return candidates
            prev = dict(self.last_query)
            self.reactor.pause(host_now + poll_s)

    def _describe_tool_detection_issue(self, candidates):
        if len(candidates) == 1 :
            return 'OK'
        elif len(candidates) == 0:
            return "All probes triggered"
        else:
            return f"Multiple probes not triggered: {[p.name for p in candidates]}"

    def _ensure_active_tool_or_fail(self, gcode):
        if self.active_probe:
            return
        active_tools = self._query_open_tools()
        if len(active_tools) != 1 :
            raise gcode.error(self._describe_tool_detection_issue(active_tools))
        self.set_active_probe(active_tools[0])

    def _detect_active_tool(self):
        active_tools = self._query_open_tools()
        if len(active_tools) == 1 :
            self.set_active_probe(active_tools[0])

    cmd_SET_ACTIVE_TOOL_PROBE_help = (
        "Set the tool probe that will act as the Z endstop (T=-1 clears selection).")
    def cmd_SET_ACTIVE_TOOL_PROBE(self, gcmd):
        probe_nr = gcmd.get_int("T")
        if probe_nr == -1:
            self.set_active_probe(None)
            return
        if (probe_nr not in self.tool_probes):
            raise gcmd.error("SET_ACTIVE_TOOL_PROBE no tool probe for tool %d" % (probe_nr))
        self.set_active_probe(self.tool_probes[probe_nr])

    cmd_DETECT_ACTIVE_TOOL_PROBE_help = "Detect which tool is active by identifying a probe that is NOT triggered"
    def cmd_DETECT_ACTIVE_TOOL_PROBE(self, gcmd):
        tool_number = gcmd.get_int("T", self.active_tool_number)
        active_tools = self._query_open_tools(tool_number)
        if len(active_tools) == 1 :
            active = active_tools[0]
            gcmd.respond_info("Found active tool probe: %s" % (active.name))
            self.set_active_probe(active)
        else:
            self.set_active_probe(None)
            gcmd.respond_info(self._describe_tool_detection_issue(active_tools))

    def get_status(self, eventtime):
        status = self.cmd_helper.get_status(eventtime)
        status['last_tools_query'] = self.last_query
        status['active_tool_number'] = self.active_tool_number
        if self.active_probe:
            status['active_tool_probe'] = self.active_probe.name
            status['active_tool_probe_x_offset'] = self.active_probe.get_offsets()[0]
            status['active_tool_probe_y_offset'] = self.active_probe.get_offsets()[1]
            status['active_tool_probe_z_offset'] = self.active_probe.get_offsets()[2]
        else:
            status['active_tool_probe'] = None
            status['active_tool_probe_x_offset'] = 0.0
            status['active_tool_probe_y_offset'] = 0.0
            status['active_tool_probe_z_offset'] = 0.0
        return status

    cmd_START_TOOL_PROBE_CRASH_DETECTION_help = "Start detecting tool crashes"
    def cmd_START_TOOL_PROBE_CRASH_DETECTION(self, gcmd):
        # Detect waits until previous print moves are finished to detect the triggers
        self.cmd_DETECT_ACTIVE_TOOL_PROBE(gcmd)
        expected_tool_number = gcmd.get_int("T", self.active_tool_number)

        if expected_tool_number is None or expected_tool_number < 0:
            raise gcmd.error("Cannot start probe crash detection - no active tool")
        if expected_tool_number != self.active_tool_number:
            raise gcmd.error("Cannot start probe crash detection - expected tool not active")
        self.crash_lasttime = 0.
        self.crash_detection_active = True

    cmd_STOP_TOOL_PROBE_CRASH_DETECTION_help = "Stop detecting tool crashes"
    def cmd_STOP_TOOL_PROBE_CRASH_DETECTION(self, gcmd):
        # Clear when current print queue is finished
        self.toolhead.register_lookahead_callback(lambda _: self.stop_crash_detection())

    def stop_crash_detection(self):
        self.crash_lasttime = 0.
        self.crash_detection_active = False

    def note_probe_triggered(self, probe, eventtime, is_triggered):
        if not self.crash_detection_active:
            return
        if probe != self.active_probe:
            return
        if is_triggered:
            self.crash_lasttime = eventtime
            self.reactor.register_callback(lambda _: self._probe_triggered_delayed(eventtime),
                                           eventtime + self.crash_mintime)
        else:
            self.crash_lasttime = 0.

    def _probe_triggered_delayed(self, expect_eventtime):
        if self.crash_lasttime != expect_eventtime:
            # The trigger was cancelled
            return
        if self.crash_detection_active:
            self.crash_detection_active = False
            self.crash_gcode.run_gcode_from_command()

# Routes commands to the selected tool probe endstop.
class EndstopRouter:
    def __init__(self, printer):
        self.active_mcu = None
        self.set_active_mcu(None)
        self._mcus = []
        self._steppers = []
        self.printer = printer

    def add_mcu(self, mcu_probe):
        self._mcus.append(mcu_probe)
        for s in self._steppers:
            mcu_probe.add_stepper(s)

    def set_active_mcu(self, mcu_probe):
        self.active_mcu = mcu_probe
        # Update Wrappers
        if self.active_mcu:
            self.get_mcu = self.active_mcu.get_mcu
            self.home_start = self.active_mcu.home_start
            self.home_wait = self.active_mcu.home_wait
            self.multi_probe_begin = self.active_mcu.multi_probe_begin
            self.multi_probe_end = self.active_mcu.multi_probe_end
            self.probe_prepare = self.active_mcu.probe_prepare
            self.probe_finish = self.active_mcu.probe_finish
        else:
            self.get_mcu = self.on_error
            self.home_start = self.on_error
            self.home_wait = self.on_error
            self.multi_probe_begin = self.on_error
            self.multi_probe_end = self.on_error
            self.probe_prepare = self.on_error
            self.probe_finish = self.on_error

    def add_stepper(self, stepper):
        self._steppers.append(stepper)
        for m in self._mcus:
            m.add_stepper(stepper)
    def get_steppers(self):
        return list(self._steppers)

    def on_error(self, *args, **kwargs):
        raise self.printer.command_error("Cannot interact with probe - no active tool probe.")
    def query_endstop(self, print_time):
        if not self.active_mcu:
            raise self.printer.command_error("Cannot query endstop - no active tool probe.")
        return self.active_mcu.query_endstop(print_time)
    def get_position_endstop(self):
        if not self.active_mcu:
            # This will get picked up by the endstop, and is static
            # Report 0 and fix up in the homing sequence
            return 0.0
        return self.active_mcu.get_position_endstop()

def load_config(config):
    return ToolProbeEndstop(config)
