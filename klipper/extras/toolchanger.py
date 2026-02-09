# Support for toolchnagers
#
# Copyright (C) 2023 Viesturs Zarins <viesturz@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import ast, bisect, traceback, logging, types
from unittest.mock import sentinel
from .probe_blind_button import ProbeBlindButton

STATUS_UNINITALIZED = 'uninitialized'
STATUS_INITIALIZING = 'initializing'
STATUS_READY = 'ready'
STATUS_CHANGING = 'changing'
STATUS_ERROR = 'error'
INIT_ON_HOME = 0
INIT_MANUAL = 1
INIT_FIRST_USE = 2
ON_AXIS_NOT_HOMED_ABORT = 0
ON_AXIS_NOT_HOMED_HOME = 1
XYZ_TO_INDEX = {'x': 0, 'X': 0, 'y': 1, 'Y': 1, 'z': 2, 'Z': 2}
INDEX_TO_XYZ = 'XYZ'
DETECT_UNAVAILABLE = -1
DETECT_ABSENT = 0
DETECT_PRESENT = 1


class GCodeSuspendHelper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.stashed_commands = []
        self._drop_current_buffer = False # just allows us to decide to discard while True
        self._is_patched = False
        self._pause_triggered = False
        self._process_depth = 0

        self.respond_to_console = (config.get('gcode_suspend_respond', 'log') == 'console')
        
        # Define the exception class dynamically to inherit from the specific gcode instance's error class
        self.ToolchangePause = type('ToolchangePause', (self.gcode.error,), {})

    def _log(self, msg):
        if self.respond_to_console:
            self.gcode.respond_info(msg)
        else:
            logging.info(msg)

    def install_patch(self):
        if self._is_patched:
            return
        
        orig_process_commands = self.gcode._process_commands

        def patched_process_commands(gcode_self, commands, need_ack=True):
            self._process_depth += 1
            try:
                # We iterate manually to allow splitting the buffer upon error
                for i, line in enumerate(commands):
                    try:
                        # Pass single line. Pass need_ack EXACTLY as received.
                        orig_process_commands([line], need_ack=need_ack)
                    
                    except self.gcode.error as e:
                        # Catch exceptions that were NOT swallowed by Klipper (need_ack=False)
                        if isinstance(e, self.ToolchangePause): # this prob very destructive →  or self._pause_triggered:
                             self._handle_pause(commands, i, e)
                             return
                        raise e
                    
                    except Exception:
                        raise

                    # Check if pause was triggered but swallowed by Klipper (need_ack=True)
                    if self._pause_triggered:
                        self._handle_pause(commands, i, self.ToolchangePause("Toolchange Pause Triggered"))
                        return
            finally:
                self._process_depth -= 1

        self.gcode._process_commands = types.MethodType(patched_process_commands, self.gcode)
        self._is_patched = True

    def _handle_pause(self, commands, current_index, exception_obj):
        if self._process_depth > 1:
            # We're inside a nested macro/command buffer; bubble to outermost.
            raise exception_obj
        if self._drop_current_buffer:
            # We are inside the failed template. We must abort this loop.
            # We raise the exception so it bubbles up to select_tool.
            raise exception_obj
        else:
            # We have bubbled up to the outer context. Stash remainder.
            remaining = commands[current_index+1:]
            if remaining:
                self.stashed_commands.extend(remaining)
                self._log(f"<details><summary>Toolchanger: Stashed {len(remaining)} commands.</summary>{'<br>'.join(self.stashed_commands)}</details>")
            # Reset triggers since we handled it
            self._pause_triggered = False
            self._log("Toolchanger: Execution suspended cleanly.")
            
            # Suppress the error to stop the loop but keep printer alive
            return

    def initiate_pause(self, reason):
        self._log("Toolchanger: Initiating pause due to: %s" % reason)
        
        # Only pause the SD card if it is actually doing something.
        # This prevents 'Print Paused' state when just running macros from console.
        sd = self.printer.lookup_object('virtual_sdcard', None)
        if sd and sd.is_active():
            self._log("Toolchanger: Pausing active SD card.")
            sd.do_pause()

        self._drop_current_buffer = True
        self._pause_triggered = True
        
        raise self.ToolchangePause(reason)

    def prepare_for_recovery(self):
        self._drop_current_buffer = False

    def replay_stash(self):
        if self.stashed_commands:
            self._log("Toolchanger: Replaying %d stashed commands." % len(self.stashed_commands))
            script = "\n".join(self.stashed_commands)
            self.stashed_commands = [] 
            self.gcode.run_script_from_command(script)

    def clear_stash(self):
        if self.stashed_commands:
            self._log("Toolchanger: Clearing stashed commands on reset.")
            self.stashed_commands = []


class Toolchanger:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.config = config
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode_move = self.printer.load_object(config, 'gcode_move')

        self.experimental_pause = config.getboolean('experimental_pause', False)
        
        self.name = config.get_name()
        self.params = get_params_dict(config)
        init_options = {'home': INIT_ON_HOME,
                        'manual': INIT_MANUAL, 'first-use': INIT_FIRST_USE}
        self.initialize_on = config.getchoice(
            'initialize_on', init_options, 'first-use')
        self.verify_tool_pickup = config.getboolean('verify_tool_pickup', True)
        self.ignore_detect_probing_events = config.getboolean(
            'ignore_detect_probing_events', True)
        self.require_tool_present = config.getboolean('require_tool_present', False)
        self.transfer_fan_speed = config.getboolean('transfer_fan_speed', True)
        self.perform_restore_move = config.getboolean('perform_restore_move', True)
        self.raise_on_error = config.getboolean('raise_on_error', True)
        self.uses_axis = config.get('uses_axis', 'xyz').lower()
        home_options = {'abort': ON_AXIS_NOT_HOMED_ABORT,
                        'home': ON_AXIS_NOT_HOMED_HOME}
        self.on_axis_not_homed = config.getchoice('on_axis_not_homed',
                                                  home_options, 'abort')
        self.initialize_gcode = self.gcode_macro.load_template(
            config, 'initialize_gcode', '')
        self.error_gcode = self.gcode_macro.load_template(config, 'error_gcode') \
            if config.get('error_gcode', None) else None
        self.on_tool_mounted_gcode = self.gcode_macro.load_template(
            config, 'on_tool_mounted_gcode', '')
        self.on_tool_removed_gcode = self.gcode_macro.load_template(
            config, 'on_tool_removed_gcode', '')
        self.default_before_change_gcode = self.gcode_macro.load_template(
            config, 'before_change_gcode', '')
        self.default_after_change_gcode = self.gcode_macro.load_template(
            config, 'after_change_gcode', '')

        # Read all the fields that might be defined on toolchanger.
        # To avoid throwing config error when no tools configured.
        config.get('pickup_gcode', None)
        config.get('dropoff_gcode', None)
        config.get('recover_gcode', None)
        config.getfloat('gcode_x_offset', None)
        config.getfloat('gcode_y_offset', None)
        config.getfloat('gcode_z_offset', None)
        config.get('t_command_restore_axis', None)
        config.get('extruder', None)
        config.get('fan', None)
        config.get_prefix_options('params_')

        self._det_btn = ProbeBlindButton(self.printer, on_change=self._on_probe_blinded_change)

        self.is_printer_ready = False 
        self._ready_timer = None

        self.status = STATUS_UNINITALIZED
        self.active_tool = None
        self.detected_tool = None
        self.has_detection = False
        self.tools = {}
        self.tool_numbers = []  # Ordered list of registered tool numbers.
        self.tool_names = []  # Tool names, in the same order as numbers.
        self.error_message = ''
        self.next_change_id = 1
        self.current_change_id = -1
        self.gcode_transform = ToolGcodeTransform()
        self.last_change_gcode_position = None
        self.last_change_gcode_offset = None
        self.last_change_restore_axis = None
        self.last_change_restore_position = None
        self.last_change_pickup_tool = None
        # Pause Support
        self.suspend_helper = None
        if self.experimental_pause:
            self.suspend_helper = GCodeSuspendHelper(config)
            self.suspend_helper.install_patch()

        self.printer.register_event_handler("gcode:command_error",
                                            self._handle_command_error)
        self.printer.register_event_handler("homing:home_rails_begin",
                                            self._handle_home_rails_begin)
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.printer.register_event_handler("klippy:ready", 
                                            self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        self.gcode.register_command("INITIALIZE_TOOLCHANGER",
                                    self.cmd_INITIALIZE_TOOLCHANGER,
                                    desc=self.cmd_INITIALIZE_TOOLCHANGER_help)
        self.gcode.register_command("SET_TOOL_TEMPERATURE",
                                    self.cmd_SET_TOOL_TEMPERATURE,
                                    desc=self.cmd_SET_TOOL_TEMPERATURE_help)
        self.gcode.register_command("SELECT_TOOL",
                                    self.cmd_SELECT_TOOL,
                                    desc=self.cmd_SELECT_TOOL_help)
        self.gcode.register_command("SELECT_TOOL_ERROR",
                                    self.cmd_SELECT_TOOL_ERROR,
                                    desc=self.cmd_SELECT_TOOL_ERROR_help)
        if not self.require_tool_present:
            self.gcode.register_command("UNSELECT_TOOL",
                                        self.cmd_UNSELECT_TOOL,
                                        desc=self.cmd_UNSELECT_TOOL_help)
        self.gcode.register_command("ENTER_DOCKING_MODE",
                                    self.cmd_ENTER_DOCKING_MODE,
                                    desc="Manually enter docking mode")
        self.gcode.register_command("EXIT_DOCKING_MODE",
                                    self.cmd_EXIT_DOCKING_MODE,
                                    desc="Manually exit docking mode")
        self.gcode.register_command("TEST_TOOL_DOCKING",
                                    self.cmd_TEST_TOOL_DOCKING,
                                    desc=self.cmd_TEST_TOOL_DOCKING_help)
        self.gcode.register_command("SET_TOOL_OFFSET", 
                                    self.cmd_SET_TOOL_OFFSET)
        self.gcode.register_command("SAVE_TOOL_OFFSET", 
                                    self.cmd_SAVE_TOOL_OFFSET)
        self.gcode.register_command("SET_TOOL_PARAMETER",
                                    self.cmd_SET_TOOL_PARAMETER)
        self.gcode.register_command("RESET_TOOL_PARAMETER",
                                    self.cmd_RESET_TOOL_PARAMETER)
        self.gcode.register_command("SAVE_TOOL_PARAMETER",
                                    self.cmd_SAVE_TOOL_PARAMETER)
        self.gcode.register_command("VERIFY_TOOL_DETECTED",
                                    self.cmd_VERIFY_TOOL_DETECTED)
        self.fan_switcher = None
        self.validate_tool_timer = None

    def require_fan_switcher(self):
        if not self.fan_switcher:
            self.fan_switcher = FanSwitcher(self, self.config)

    def _handle_home_rails_begin(self, homing_state, rails):
        if self.initialize_on == INIT_ON_HOME and self.status == STATUS_UNINITALIZED:
            self.initialize(self.detected_tool)

    def _handle_connect(self):
        self.status = STATUS_UNINITALIZED
        self.active_tool = None
        self.gcode_transform.next_transform = self.gcode_move.set_move_transform(
            self.gcode_transform, force=True)
        if self.suspend_helper:
            self.suspend_helper.clear_stash()

    def _handle_command_error(self):
        self.status = STATUS_UNINITALIZED
        self.active_tool = None
        self.gcode_transform.tool = None

    def _handle_shutdown(self):
        self.status = STATUS_UNINITALIZED
        self.active_tool = None
        self.gcode_transform.tool = None
        self.is_printer_ready = False
        r = self.printer.get_reactor()
        if self._ready_timer is not None:
            r.update_timer(self._ready_timer, r.NEVER)
            self._ready_timer = None
        if self.suspend_helper:
            self.suspend_helper.clear_stash()

    def _handle_ready(self):
        def _ready_after_delay(eventtime):
            self.is_printer_ready = True
            if self.has_detection:
                try:
                    self.note_detect_change(None)
                except Exception:
                    pass
            self._ready_timer = None
            return self.printer.get_reactor().NEVER
        r = self.printer.get_reactor()
        if self._ready_timer is None: # this doesnt work at all
            self._ready_timer = r.register_timer(_ready_after_delay, r.monotonic() + 0.1)

    def get_status(self, eventtime):
        return {**self.params,
                'name': self.name,
                'status': self.status,
                'tool': self.active_tool.name if self.active_tool else None,
                'tool_number': self.active_tool.tool_number if self.active_tool else -1,
                'detected_tool': self.detected_tool.name if self.detected_tool else None,
                'detected_tool_number': self.detected_tool.tool_number if self.detected_tool else -1,
                'tool_numbers': self.tool_numbers,
                'tool_names': self.tool_names,
                'has_detection': self.has_detection,
                }

    def assign_tool(self, tool, number, prev_number, replace=False):
        if number in self.tools and not replace:
            raise Exception('Duplicate tools with number %s' % (number,))
        if prev_number in self.tools:
            del self.tools[prev_number]
            self.tool_numbers.remove(prev_number)
            self.tool_names.remove(tool.name)
        self.tools[number] = tool
        position = bisect.bisect_left(self.tool_numbers, number)
        self.tool_numbers.insert(position, number)
        self.tool_names.insert(position, tool.name)

        self.has_detection = any([t.detect_state != DETECT_UNAVAILABLE for t in self.tools.values()])
        all_detection = all([t.detect_state != DETECT_UNAVAILABLE for t in self.tools.values()])
        if self.has_detection and not all_detection:
            raise self.config.error("Some tools missing detection pin")
        elif not self.has_detection and (self.config.get('on_tool_mounted_gcode', False) or \
                                        self.config.get('on_tool_removed_gcode', False)):
            #TODO dont raise config error at runtime!
            raise self.config.error('on_tool_mounted_gcode or on_tool_removed_gcode require tool detection')
        
    cmd_INITIALIZE_TOOLCHANGER_help = "Initialize the toolchanger"

    def cmd_INITIALIZE_TOOLCHANGER(self, gcmd):
        tool = self.gcmd_tool(gcmd, self.detected_tool)  # type: ignore
        was_error  = self.status == STATUS_ERROR
        self.initialize(tool)
        if was_error and gcmd.get_int("RECOVER", default=0) == 1:
            if not tool:
                raise gcmd.error("Cannot recover, no tool")
            self._recover_position(gcmd, tool)

    cmd_SELECT_TOOL_help = 'Select active tool'
    def cmd_SELECT_TOOL(self, gcmd):
        tool = self.gcmd_tool(gcmd)
        restore_axis = gcmd.get('RESTORE_AXIS', tool.t_command_restore_axis)  # type: ignore
        self.select_tool(gcmd, tool, restore_axis)

    cmd_SET_TOOL_TEMPERATURE_help = 'Set temperature for tool'

    def cmd_SET_TOOL_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float('TARGET', 0.)
        wait = gcmd.get_int('WAIT', 0) == 1
        tool = self._get_tool_from_gcmd(gcmd)
        if not tool.extruder:
            raise gcmd.error(
                "SET_TOOL_TEMPERATURE: No extruder specified for tool %s" % (
                    tool.name))
        heaters = self.printer.lookup_object('heaters')
        heaters.set_temperature(tool.extruder.get_heater(), temp, wait)

    def _get_tool_from_gcmd(self, gcmd):
        cmd_name  = gcmd.get_command()
        tool_name = gcmd.get('TOOL', None)
        tool_nr   = gcmd.get_int('T', None)
        if tool_name is not None:
            tool = self.printer.lookup_object(tool_name, None)
            if tool is None:
                raise gcmd.error("%s: TOOL: '%s' not found" % (cmd_name, tool_name))          
        elif tool_nr is not None:
            tool = self.lookup_tool(tool_nr)
            if tool is None:
                raise gcmd.error("%s: T%d not found" % (cmd_name, tool_nr))
        else:
            tool = self.active_tool
            if tool is None:
                raise gcmd.error("%s: No tool specified and no active tool" % (cmd_name))
        return tool

    cmd_SELECT_TOOL_ERROR_help = "Abort tool change and mark the active toolchanger as failed"
    def cmd_SELECT_TOOL_ERROR(self, gcmd):
        if self.status != STATUS_CHANGING and self.status != STATUS_INITIALIZING:
            gcmd.respond_info(
                'SELECT_TOOL_ERROR called while not selecting, doing nothing')
            return
        message = gcmd.get('MESSAGE', '')
        self._process_error(gcmd.error, message)

    cmd_UNSELECT_TOOL_help = "Unselect active tool without selecting a new one"
    def cmd_UNSELECT_TOOL(self, gcmd):
        if not self.active_tool:
            return
        restore_axis = gcmd.get('RESTORE_AXIS',
                                self.active_tool.t_command_restore_axis)
        self.select_tool(gcmd, None, restore_axis)

    def cmd_ENTER_DOCKING_MODE(self, gcmd):
        if self.status == STATUS_UNINITALIZED and self.initialize_on == INIT_FIRST_USE:
            self.initialize(self.detected_tool)
        if self.status != STATUS_READY:
            raise gcmd.error(
                "Cannot enter docking mode, toolchanger status is %s, reason: %s" % (self.status, self.error_message))
        self.status = STATUS_CHANGING
        self._save_state("", None)
        self._set_toolchange_transform()

    def cmd_EXIT_DOCKING_MODE(self, gcmd):
        if self.status != STATUS_CHANGING:
            raise gcmd.error(
                "Cannot exit docking mode, toolchanger status is %s, reason: %s" % (self.status, self.error_message))

        self._restore_state_and_transform(self.active_tool)
        self.status = STATUS_READY

    cmd_TEST_TOOL_DOCKING_help = "Unselect active tool and select it again"
    def cmd_TEST_TOOL_DOCKING(self, gcmd):
        if not self.active_tool:
            raise gcmd.error("Cannot test tool, no active tool")
        restore_axis = gcmd.get('RESTORE_AXIS',
                                self.active_tool.t_command_restore_axis)
        self.test_tool_selection(gcmd, restore_axis)

    def initialize(self, select_tool=None):
        if self.status == STATUS_CHANGING:
            raise Exception('Cannot initialize while changing tools')

        # Initialize may be called from within the intialize gcode
        # to set active tool without performing a full change
        should_run_initialize = self.status != STATUS_INITIALIZING

        extra_context = {
            'dropoff_tool': None,
            'pickup_tool': select_tool.name if select_tool else None,
        }
        try:
            if should_run_initialize:
                self.status = STATUS_INITIALIZING
                self.run_gcode('initialize_gcode', self.initialize_gcode, extra_context)

            if select_tool or self.has_detection:
                self._configure_toolhead_for_tool(select_tool)
                if select_tool:
                    self.run_gcode('after_change_gcode', select_tool.after_change_gcode, extra_context)
                    self.gcode_transform.tool = select_tool
                if self.require_tool_present and self.active_tool is None:
                    raise self.gcode.error(
                        '%s failed to initialize, require_tool_present set and no tool present after initialization' % (
                            self.name,))

            if should_run_initialize:
                if self.status == STATUS_INITIALIZING:
                    self.status = STATUS_READY
                    self.gcode.respond_info('%s initialized, active %s' %
                                            (self.name,
                                            self.active_tool.name if self.active_tool else None))
                else:
                    raise self.gcode.error('%s failed to initialize, error: %s' %
                                        (self.name, self.error_message))
        except Exception as e:
            self.error_message = str(e)
            self.status = STATUS_UNINITALIZED
            raise self.gcode.error('%s failed to initialize, error: %s' % (self.name, str(e))) from e


    def select_tool(self, gcmd, tool, restore_axis):
        if self.status == STATUS_UNINITALIZED and self.initialize_on == INIT_FIRST_USE:
            self.initialize(self.detected_tool)
        if self.status != STATUS_READY:
            raise gcmd.error(
                "Cannot select tool, toolchanger status is %s, reason: %s" % (self.status, self.error_message))

        if self.active_tool == tool:
            gcmd.respond_info('%s already selected' % tool.name if tool else None)
            return
        # should we allow disconnected tools to be unselected if they're disconnected? probably, right? not guarding it.
        if getattr(tool, 'is_disconnected', False):
            raise gcmd.error('cannot select tool, %s is disconnected' % (tool.name,))
        this_change_id = self.next_change_id
        self.next_change_id += 1
        self.current_change_id = this_change_id

        try:
            self.ensure_homed(gcmd)
            self.status = STATUS_CHANGING
            self._save_state(restore_axis, tool)

            # Read optional XYZ overrides
            overrides = {ax: gcmd.get_float(ax, None) for ax in 'XYZ'}
            overrides = {ax: v for ax, v in overrides.items() if v is not None}

            restore_position = list(self.last_change_gcode_position)
            if overrides:  # If provided, augment restore_axis with those axes
                existing = (restore_axis or '').lower()
                provided = ''.join(a for a in 'xyz' if overrides.get(a.upper()) is not None)
                restore_axis = ''.join(a for a in 'xyz' if (a in existing) or (a in provided))
                for ax, val in overrides.items():  # Apply the overrides to the restore position
                    restore_position[XYZ_TO_INDEX[ax]] = val

            self.last_change_restore_axis = restore_axis
            self.last_change_restore_position = restore_position

            start_position = self._position_with_tool_offset(self.last_change_gcode_position, tool)
            restore_position_with_tool = self._position_with_tool_offset(restore_position, tool)
            extra_context = {
                'dropoff_tool': self.active_tool.name if self.active_tool else None,
                'pickup_tool': tool.name if tool else None,
                'start_position': self._position_to_xyz(start_position, 'xyz'),
                'restore_position': self._position_to_xyz(restore_position_with_tool, restore_axis or ''),
            }

            before_change_gcode = self.active_tool.before_change_gcode if self.active_tool else self.default_before_change_gcode
            self.run_gcode('before_change_gcode', before_change_gcode, extra_context)
            self._set_toolchange_transform()

            if self.active_tool:
                self.run_gcode('tool.dropoff_gcode',
                               self.active_tool.dropoff_gcode, extra_context)

            self._configure_toolhead_for_tool(tool)
            if tool is not None:
                self.run_gcode('tool.pickup_gcode',
                               tool.pickup_gcode, extra_context)
                if self.has_detection and self.verify_tool_pickup:
                    self.validate_detected_tool(tool, respond_info=gcmd.respond_info, raise_error=gcmd.error)
                self.run_gcode('after_change_gcode',
                               tool.after_change_gcode, extra_context)

            perform_restore = tool.perform_restore_move if tool is not None else self.perform_restore_move
            self._restore_state_and_transform(tool, perform_restore_move=perform_restore)
            self.status = STATUS_READY
            if tool:
                gcmd.respond_info('Selected tool %s (%s)' % (str(tool.tool_number), tool.name))
            else:
                gcmd.respond_info('Tool unselected')
            self.current_change_id = -1

        except self.gcode.error or gcmd.error as e: # idk theyre technically the same but im paranoid 
            # Experimental Pause Handling:
            # Check if this error was actually a pause initiated by us.
            # this can **only** come from validate/select_tool_error
            if self.suspend_helper and isinstance(e, self.suspend_helper.ToolchangePause): 
                if self.suspend_helper.respond_to_console:  # type: ignore
                    gcmd.respond_info("Toolchanger: Suspend caught in select_tool. Preparing recovery.")
                self.suspend_helper.prepare_for_recovery() # type: ignore
                self.current_change_id = -1
                raise # Re-raise so the patch can catch it in outer loop and Stash.
                      # "raise" ie tell virtual sd card were okay but need a pause

            # --------------------------------------------------------------------
            # Standard error handling, only from validate or SELECT_TOOL_ERROR
            if self.status == STATUS_ERROR: # gets set by validate detected, or select tool error
                if self.raise_on_error: # fuck it
                    raise # pass # handled as a default error
                else:
                    pass
            else:
                # regular gcmd errors end up in here, for example action_raise_error.
                self.current_change_id = -1
                self.status = STATUS_UNINITALIZED # test, imo this would be "correct"? 
                # because we raise the error up but mark us as "something wrong" 
                raise

    def _process_error(self, raise_error, message):
        self.status = STATUS_ERROR
        self.error_message = message
        is_inside_toolchange = self.current_change_id != -1
        captured_exc = None
        self.current_change_id = -1
        if self.error_gcode:
            extra_context = {}
            if is_inside_toolchange:
                start_position = self._position_with_tool_offset(
                    self.last_change_gcode_position, self.last_change_pickup_tool)
                restore_base = self.last_change_restore_position or self.last_change_gcode_position
                restore_position = self._position_with_tool_offset(
                    restore_base, self.last_change_pickup_tool)
                extra_context = {
                    'start_position': self._position_to_xyz(start_position, "xyz"),
                    'restore_position': self._position_to_xyz(
                        restore_position, self.last_change_restore_axis or ''),
                    'pickup_tool': self.last_change_pickup_tool,
                }
                # Restore gcode state, but do not move. Prepare for error_gcode to run pause and capture the state for resume.
                self.gcode.run_script_from_command(
                    "RESTORE_GCODE_STATE NAME=_toolchange_state MOVE=0"
                )
                
                try: # may itself raise an error? how to handle that?
                    self.run_gcode('error_gcode', self.error_gcode, extra_context)
                except Exception as e:
                    captured_exc  = e

            if is_inside_toolchange:
                # HACKY HACKY HACKY
                # Manually transfer over before toolchange position to paused gcode state, Restore/Save looses that.
                pause_state = self.gcode_move.saved_states.get('PAUSE_STATE', None)
                if pause_state and self.last_change_pickup_tool:
                    pause_state['last_position'] = self.last_change_gcode_position

        # Bail out rest of the gcmd execution.
        if self.experimental_pause and is_inside_toolchange and self.suspend_helper:
            self.suspend_helper.initiate_pause(message) # raises our special error type
        if captured_exc is not None:
            raise captured_exc 
        if raise_error:
            raise raise_error(message)

    def _recover_position(self, gcmd, tool):
        start_position = self._position_with_tool_offset(self.last_change_gcode_position, tool)
        restore_base = self.last_change_restore_position or self.last_change_gcode_position
        restore_position = self._position_with_tool_offset(restore_base, tool)
        extra_context = {
            'pickup_tool': tool.name if tool else None,
            'start_position': self._position_to_xyz(start_position, "xyz"),
            'restore_position': self._position_to_xyz(restore_position, self.last_change_restore_axis or ''),
        }
        self.run_gcode('recover_gcode', tool.recover_gcode, extra_context)
        perform_restore = tool.perform_restore_move if tool is not None else self.perform_restore_move
        self._restore_state_and_transform(tool, perform_restore_move=perform_restore)

        if self.suspend_helper:
            self.suspend_helper.replay_stash()

    def test_tool_selection(self, gcmd, restore_axis):
        if self.status != STATUS_CHANGING:
            raise self.gcode.error(
                "Docking test requires STATUS_CHANGING, status is %s" % (self.status,))
        tool = self.active_tool
        if not tool:
            raise gcmd.error("Cannot test tool, no active tool")

        gcode_position = list(self.gcode_move.get_status()['gcode_position'])
        start_position = self._position_with_tool_offset(gcode_position, None)
        extra_context = {
            'dropoff_tool': tool.name if tool else None,
            'pickup_tool': tool.name if tool else None,
            'start_position': self._position_to_xyz(start_position,'xyz'),
            'restore_position': self._position_to_xyz(start_position, restore_axis),
        }

        self.run_gcode('tool.dropoff_gcode',
                       self.active_tool.dropoff_gcode, extra_context)  # type: ignore
        self.run_gcode('tool.pickup_gcode',
                       tool.pickup_gcode, extra_context)
        self._restore_axis(gcode_position, restore_axis)
        gcmd.respond_info('Tool testing done')

    def lookup_tool(self, number):
        return self.tools.get(number, None)

    def get_selected_tool(self):
        return self.active_tool

    def note_detect_change(self, _tool=None):
        if not self.is_printer_ready or not self.has_detection:
            return
        detected = None
        detected_names = []
        for t in self.tools.values():
            if getattr(t, 'is_disconnected', False):
                continue
            if t.detect_state == DETECT_PRESENT:
                detected = t
                detected_names.append(t.name)
        if len(detected_names) > 1:
            detected = None

        if not self.ignore_detect_probing_events:
            self.detected_tool = detected
        self._det_btn.note_change(detected)

    def _on_probe_blinded_change(self, last, new):
        if self.ignore_detect_probing_events:
            self.detected_tool = new
        if self.status in (STATUS_CHANGING, STATUS_INITIALIZING):
            return
        if new:
            self.run_gcode('on_tool_mounted_gcode', new.on_tool_mounted_gcode, {'detected_tool': new, 'removed_tool': None})
        elif last:
            self.run_gcode('on_tool_removed_gcode', last.on_tool_removed_gcode, {'detected_tool': None, 'removed_tool': last})

    def require_detected_tool(self, respond_info):
        if self.detected_tool is not None:
            return self.detected_tool
        detected = None
        detected_names = []
        for tool in self.tools.values():
            if tool.detect_state == DETECT_PRESENT:
                detected = tool
                detected_names.append(tool.name)
        if len(detected_names) > 1:
            respond_info("Multiple tools detected: %s" % (detected_names,))
        if detected is None:
            respond_info("No tool detected")
        return detected

    def validate_detected_tool(self, expected, respond_info, raise_error):
        actual = self.require_detected_tool(respond_info)
        if actual != expected:
            expected_name = expected.name if expected else "None"
            actual_name = actual.name if actual else "None"
            message = "Expected %s but active is %s" % (expected_name, actual_name)
            self._process_error(raise_error, message)

    def cmd_VERIFY_TOOL_DETECTED(self, gcmd):
        self._ensure_toolchanger_ready(gcmd)
        expected = self.gcmd_tool(gcmd, self.active_tool)  # type: ignore
        if not self.has_detection:
            raise gcmd.error("VERIFY_TOOL_DETECTED needs tool detection to be set up.")

        toolhead = self.printer.lookup_object('toolhead')
        reactor  = self.printer.get_reactor()
        def _kin_flush_delay():
            # klipper newer (~august 2025?)
            motion_queuing = self.printer.lookup_object('motion_queuing', None)
            if motion_queuing and hasattr(motion_queuing, 'get_kin_flush_delay'):
                return motion_queuing.get_kin_flush_delay()
            # kalico/old default
            return getattr(toolhead, 'kin_flush_delay', 0.0)

        if gcmd.get_int("ASYNC", 0) == 1:
            if self.error_gcode is None:
                raise gcmd.error("VERIFY_TOOL_DETECTED ASYNC=1 needs error_gcode to be defined")
            # Cancel any previously scheduled reactor timer
            if self.validate_tool_timer is not None:
                reactor.unregister_timer(self.validate_tool_timer)
                self.validate_tool_timer = None
            self._vtd_seq = getattr(self, '_vtd_seq', 0) + 1
            seq = self._vtd_seq # Generation counter so old callbacks/timers do nothing

            window = gcmd.get_float("WINDOW", 0.005, minval=0.0)

            def _timer_handler(eventtime):
                if seq != getattr(self, '_vtd_seq', None):
                    return reactor.NEVER

                et = eventtime
                deadline = et + window # Micro-poll up to 10 ms for late button messages (idk man, paranoia)
                while self.detected_tool != expected and et < deadline:
                    et = reactor.pause(et + 0.001)

                # clear handle and validate (no raise, error_gcode handles failure)
                self.validate_tool_timer = None
                self.gcode.respond_info(f"micro-poll waited {(et - eventtime)*1000:.1f} ms; detected={self.detected_tool == expected}")
                self.validate_detected_tool(expected, respond_info=gcmd.respond_info, raise_error=None)
                if self.detected_tool != expected: 
                    toolhead.lookahead.reset() # we fucked up
                    self.gcode.run_script_from_command("M112")
                return reactor.NEVER

            def _lookahead_cb(print_time):
                if seq != getattr(self, '_vtd_seq', None):
                    return
                now     = reactor.monotonic()
                mcu_now = toolhead.mcu.estimated_print_time(now)
                fire_at = now + max(0.0, print_time + _kin_flush_delay() - mcu_now)

                if self.validate_tool_timer is not None:
                    reactor.unregister_timer(self.validate_tool_timer)
                    self.validate_tool_timer = None
                self.validate_tool_timer = reactor.register_timer(_timer_handler, fire_at)

            # schedule after currently queued motion truly finishes
            toolhead.register_lookahead_callback(_lookahead_cb)
            return
        else:
            # poll until detected, or 0.5 s after true motion end passes
            poll_s, timeout_s = 0.0025, 0.5
            anchor_pt = toolhead.get_last_move_time() + _kin_flush_delay()  # print_time anchor
            eventtime = reactor.monotonic()
            deadline  = None  # reactor-time deadline set once MCU crosses anchor

            while True: # ARE WE THERE YET;  ARE WE THERE YET;  ARE WE THERE YET;  ARE WE THERE YET;  ARE WE THERE YET
                if self.detected_tool == expected:
                    self.validate_detected_tool(expected, respond_info=gcmd.respond_info, raise_error=gcmd.error)
                    return
                mcu_now = toolhead.mcu.estimated_print_time(eventtime)
                if deadline is None and mcu_now >= anchor_pt:
                    deadline = eventtime + timeout_s
                if deadline is not None and eventtime >= deadline:
                    self.validate_detected_tool(expected, respond_info=gcmd.respond_info, raise_error=gcmd.error)
                    return
                eventtime = reactor.pause(eventtime + poll_s)

    def _configure_toolhead_for_tool(self, tool):
        if self.active_tool:
            self.active_tool.deactivate()
        self.active_tool = tool
        if self.active_tool:
            self.active_tool.activate()

    def _position_to_xyz(self, position, axis):
        if len(position) < 3:
            raise Exception(f"Invalid position: {position}")
        result = {}
        for i in axis:
            if i in XYZ_TO_INDEX:
                index = XYZ_TO_INDEX[i]
                result[INDEX_TO_XYZ[index]] = position[index]
        return result

    # Position with tool transforms applied. To be used while toolchanging to get a gcode position.
    def _position_with_tool_offset(self, position, tool):
        if len(position) < 3:
            raise Exception(f"Invalid position: {position}")
        result = []
        for i in range(3):
            v = position[i]
            if self.last_change_gcode_offset is not None:
                v += self.last_change_gcode_offset[i]
            if tool:
                if i == 0:
                    v += tool.gcode_x_offset
                elif i == 1:
                    v += tool.gcode_y_offset
                elif i == 2:
                    v += tool.gcode_z_offset
            result.append(v)
        result.extend(position[3:])
        return result

    def _save_state(self, restore_axis, tool):
        """What is going on here:
         - toolhead position - the position of the toolhead mount relative to homing sensors.
         - gcode position - the position of the nozzle, relative to the bed;
             since each tool has a slightly different geometry, each tool has a set of gcode offsets that determine the delta.
        Normally gcode commands use gcode position, but that can mean different toolhead positions depending on
        which tool is mounted, making tool changes unreliable.
        To solve that, during toolchange Gcode offsets are set to zero and the gcode moves directly work with toolhead position.
        And the nozzle location will deviate for each tool.

        To restore the new tool's nozzle to where the previous tool left off, the restore position is manually computed in the code below.
        """
        gcode_status = self.gcode_move.get_status()

        self.gcode.run_script_from_command("SAVE_GCODE_STATE NAME=_toolchange_state")
        self.last_change_pickup_tool = tool
        self.last_change_gcode_position = list(gcode_status['gcode_position'])
        self.last_change_gcode_offset = gcode_status['homing_origin']
        self.last_change_restore_axis = restore_axis
        self.last_change_restore_position = None

    def _set_toolchange_transform(self):
        self.gcode_transform.tool = None
        self.gcode_move.reset_last_position()
        self.gcode.run_script_from_command("SET_GCODE_OFFSET X=0.0 Y=0.0 Z=0.0")

    def _restore_state_and_transform(self, tool, perform_restore_move=True):
        self.gcode_transform.tool = tool
        self.gcode_move.reset_last_position()
        self.gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=_toolchange_state MOVE=0")
        self.last_change_gcode_offset = None
        if perform_restore_move and self.last_change_restore_axis:
            restore_position = self.last_change_restore_position or self.last_change_gcode_position
            self._restore_axis(restore_position, self.last_change_restore_axis)
            self.gcode.run_script_from_command("RESTORE_GCODE_STATE NAME=_toolchange_state MOVE=0")

    def _restore_axis(self, position, axis):
        if not axis:
            return
        pos = self._position_with_tool_offset(position, None)
        self.gcode.run_script_from_command("G90")
        self.gcode_move.cmd_G1(self.gcode.create_gcode_command(
            "G0", "G0", self._position_to_xyz(pos, axis)))

    def run_gcode(self, name, template, extra_context):
        curtime = self.printer.get_reactor().monotonic()
        context = {
            **template.create_template_context(),
            'tool': self.active_tool.get_status(
                curtime) if self.active_tool else {},
            'toolchanger': self.get_status(curtime),
            **extra_context,
        }
        template.run_gcode_from_command(context)
        
    def cmd_SET_TOOL_OFFSET(self, gcmd):
        tool = self._get_tool_from_gcmd(gcmd)
        _x = gcmd.get_float("X", None)
        _y = gcmd.get_float("Y", None)
        _z = gcmd.get_float("Z", None)
        if _x is None and _y is None and _z is None: 
            raise gcmd.error('SET_TOOL_OFFSET requires atleast one paramter of X, Y, Z')
        tool.gcode_x_offset = x = gcmd.get_float("X", tool.gcode_x_offset)
        tool.gcode_y_offset = y = gcmd.get_float("Y", tool.gcode_y_offset)
        tool.gcode_z_offset = z = gcmd.get_float("Z", tool.gcode_z_offset)
        if tool is self.active_tool:
            self.gcode_transform.tool = tool
            self.gcode_move.reset_last_position()
        gcmd.respond_info('Tool %s (%s) offset is now X=%.3f Y=%.3f Z=%.3f' % (str(tool.tool_number), tool.name, x, y, z))

    def cmd_SAVE_TOOL_OFFSET(self, gcmd):
        tool = self._get_tool_from_gcmd(gcmd)
        x = gcmd.get_float("X", tool.gcode_x_offset)
        y = gcmd.get_float("Y", tool.gcode_y_offset)
        z = gcmd.get_float("Z", tool.gcode_z_offset)
        configfile = self.printer.lookup_object('configfile')
        configfile.set(tool.name, 'gcode_x_offset', x)
        configfile.set(tool.name, 'gcode_y_offset', y)
        configfile.set(tool.name, 'gcode_z_offset', z)
        
    def cmd_SET_TOOL_PARAMETER(self, gcmd):
        tool = self._get_tool_from_gcmd(gcmd)
        name = gcmd.get("PARAMETER")
        if name in tool.params and name not in tool.original_params:
            tool.original_params[name] = tool.params[name]
        value = ast.literal_eval(gcmd.get("VALUE"))
        tool.params[name] = value

    def cmd_RESET_TOOL_PARAMETER(self, gcmd):
        tool = self._get_tool_from_gcmd(gcmd)
        name = gcmd.get("PARAMETER")
        if name in tool.original_params:
            tool.params[name] = tool.original_params[name]

    def cmd_SAVE_TOOL_PARAMETER(self, gcmd):
        tool = self._get_tool_from_gcmd(gcmd)
        name = gcmd.get("PARAMETER")
        if name not in tool.params:
            raise gcmd.error('Tool does not have parameter %s' % (name))
        configfile = self.printer.lookup_object('configfile')
        configfile.set(tool.name, name, tool.params[name])

    def ensure_homed(self, gcmd):
        if not self.uses_axis:
            return

        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.printer.get_reactor().monotonic()
        homed = toolhead.get_kinematics().get_status(curtime)['homed_axes']
        needs_homing = any(axis not in homed for axis in self.uses_axis)
        if not needs_homing:
            return

        # Wait for current moves to finish to ensure we are up-to-date
        # This stalls the movement pipeline, so only do that if homing is needed
        toolhead.wait_moves()
        curtime = self.printer.get_reactor().monotonic()
        homed = toolhead.get_kinematics().get_status(curtime)['homed_axes']
        axis_to_home = list(filter(lambda a: a not in homed, self.uses_axis))
        if not axis_to_home:
            return

        if self.on_axis_not_homed == ON_AXIS_NOT_HOMED_ABORT:
            raise gcmd.error(
                "Cannot perform toolchange, axis not homed. Required: %s, homed: %s" % (
                    self.uses_axis, homed))
        # Home the missing axis
        axis_str = " ".join(axis_to_home).upper()
        gcmd.respond_info('Homing%s before toolchange' % (axis_str,))
        self.gcode.run_script_from_command("G28 %s" % (axis_str,))

        # Check if now we are good
        toolhead.wait_moves()
        curtime = self.printer.get_reactor().monotonic()
        homed = toolhead.get_kinematics().get_status(curtime)['homed_axes']
        axis_to_home = list(filter(lambda a: a not in homed, self.uses_axis))
        if axis_to_home:
            raise gcmd.error(
                "Cannot perform toolchange, required axis still not homed after homing move. Required: %s, homed: %s" % (
                    self.uses_axis, homed))

    class sentinel:
        pass

    def gcmd_tool(self, gcmd, default=sentinel, extra_number_arg=None):
        tool_name = gcmd.get('TOOL', None)
        tool_number = gcmd.get_int('T', None)
        if tool_number is None and extra_number_arg:
            tool_number = gcmd.get_int(extra_number_arg, None)
        tool = None
        if tool_name:
            tool = self.printer.lookup_object(tool_name, None)
        if tool is None and tool_number is not None:
            tool = self.lookup_tool(tool_number)
            if not tool:
                raise gcmd.error('Tool #%d is not assigned' % (tool_number))
        if tool is None:
            if default == sentinel:
                raise gcmd.error('Missing TOOL=<name> or T=<number>')
            tool = default
        return tool

    def _ensure_toolchanger_ready(self, gcmd):
        if self.status not in [STATUS_READY, STATUS_CHANGING]:
            raise gcmd.error("VERIFY_TOOL_DETECTED: toolchanger not ready: status = %s" % (self.status,))
            
class FanSwitcher:
    def __init__(self, toolchanger, config):
        self.toolchanger = toolchanger
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.config = config
        self.has_multi_fan = bool(config.get_prefix_sections('multi_fan'))
        self.has_printer_fan = bool(config.has_section('fan'))
        self.pending_speed = None
        self.active_fan = None
        self.transfer_fan_speed = toolchanger.transfer_fan_speed
        if self.has_printer_fan:
            raise config.error("Cannot use tool fans together with [fan], use [fan_generic] for tool fans.")
        if not self.has_multi_fan and not self.has_printer_fan:
            self.gcode.register_command("M106", self.cmd_M106)
            self.gcode.register_command("M107", self.cmd_M107)

    def activate_fan(self, fan):
        if self.has_multi_fan:
            # Legacy multi-fan support
            self.gcode.run_script_from_command("ACTIVATE_FAN FAN='%s'" % (fan.name,))
            return
        if self.active_fan == fan or not self.transfer_fan_speed:
            return

        speed_to_set = self.pending_speed
        if self.active_fan:
            speed_to_set = self.active_fan.get_status(0)['speed']
            self.gcode.run_script_from_command("SET_FAN_SPEED FAN='%s' SPEED=%s" % (self.active_fan.fan_name, 0.0))
        self.active_fan = fan
        if speed_to_set is not None:
            if self.active_fan:
                self.pending_speed = None
                self.gcode.run_script_from_command(
                    "SET_FAN_SPEED FAN='%s' SPEED=%s" % (self.active_fan.fan_name, speed_to_set))
            else:
                self.pending_speed = speed_to_set

    def cmd_M106(self, gcmd):
        tool = self.toolchanger.gcmd_tool(gcmd, default=self.toolchanger.active_tool, extra_number_arg='P')
        speed = gcmd.get_float('S', 255., minval=0.) / 255.
        self.set_speed(speed, tool)

    def cmd_M107(self, gcmd):
        tool = self.toolchanger.gcmd_tool(gcmd, default=self.toolchanger.active_tool, extra_number_arg='P')
        self.set_speed(0.0, tool)

    def set_speed(self, speed, tool):
        if tool and tool.fan:
            self.gcode.run_script_from_command("SET_FAN_SPEED FAN='%s' SPEED=%s" % (tool.fan.fan_name, speed))
        else:
            self.pending_speed = speed

# Helper class for applying tool offset
class ToolGcodeTransform:
    def __init__(self):
        self.next_transform = None
        self.tool = None

    def move(self, newpos, speed):
        if not self.tool:
            return self.next_transform.move(newpos, speed)
        transformed_pos = [newpos[0] + self.tool.gcode_x_offset, newpos[1] + self.tool.gcode_y_offset, newpos[2] + self.tool.gcode_z_offset] + newpos[3:]
        return self.next_transform.move(transformed_pos, speed)

    def get_position(self):
        base_pos = self.next_transform.get_position()
        if not self.tool:
            return base_pos
        return [base_pos[0] - self.tool.gcode_x_offset, base_pos[1] - self.tool.gcode_y_offset, base_pos[2] - self.tool.gcode_z_offset] + base_pos[3:]

def get_params_dict(config):
    result = {}
    for option in config.get_prefix_options('params_'):
        try:
            result[option] = ast.literal_eval(config.get(option))
        except (ValueError, SyntaxError) as e:
            raise config.error(
                "Option '%s' in section '%s' is not a valid literal: %s"
                % (option, config.get_name(), str(e))
            )
    return result


def load_config(config):
    return Toolchanger(config)


def load_config_prefix(config):
    return Toolchanger(config)
