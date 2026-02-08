# Support for toolchnagers
#
# Copyright (C) 2023 Viesturs Zarins <viesturz@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging

from . import toolchanger

class Tool:

    def __init__(self, config):
        self.printer = config.get_printer()
        self.params = config.get_prefix_options('params_')
        self.gcode_macro = self.printer.load_object(config, 'gcode_macro')

        self.name = config.get_name()
        toolchanger_name = config.get('toolchanger', 'toolchanger')
        self.main_toolchanger = self.printer.load_object(config, 'toolchanger')
        self.toolchanger = self.printer.load_object(config, toolchanger_name)
        self.pickup_gcode = self.gcode_macro.load_template(
            config, 'pickup_gcode', self._config_get(config, 'pickup_gcode', ''))
        self.dropoff_gcode = self.gcode_macro.load_template(
            config, 'dropoff_gcode', self._config_get(config, 'dropoff_gcode', ''))
        self.before_change_gcode = self.gcode_macro.load_template(
            config, 'before_change_gcode', self._config_get(config, 'before_change_gcode', ''))
        self.after_change_gcode = self.gcode_macro.load_template(
            config, 'after_change_gcode', self._config_get(config, 'after_change_gcode', ''))
        self.on_tool_mounted_gcode = self.gcode_macro.load_template(
            config, 'on_tool_mounted_gcode', self._config_get(config, 'on_tool_mounted_gcode', ''))
        self.on_tool_removed_gcode = self.gcode_macro.load_template(
            config, 'on_tool_removed_gcode', self._config_get(config, 'on_tool_removed_gcode', ''))
        self.recover_gcode = self.gcode_macro.load_template(
            config, 'recover_gcode', self._config_get(config, 'recover_gcode', ''))
        self.gcode_x_offset = self._config_getfloat(
            config, 'gcode_x_offset', 0.0)
        self.gcode_y_offset = self._config_getfloat(
            config, 'gcode_y_offset', 0.0)
        self.gcode_z_offset = self._config_getfloat(
            config, 'gcode_z_offset', 0.0)
        self.params = {**self.toolchanger.params, **toolchanger.get_params_dict(config)}
        self.original_params = {}
        self.extruder_name = self._config_get(config, 'extruder', None)

        self.detect_state       = toolchanger.DETECT_UNAVAILABLE
        self.flip_detect_state  = True
        # NOTE: there probably is some really obscure case where extruder and detect pin aren't on the same MCU 
        # and this logic becomes flawed, but at that point theres something else really wrong with you
        self.detect_mcu         = None
        self.is_disconnected    = False
        _detect_pin_name = config.get('detection_pin', None)
        if _detect_pin_name:
            self._register_button(config, _detect_pin_name)
            
        self.extruder_stepper_name = self._config_get(config, 'extruder_stepper', None)
        self.extruder = None
        self.extruder_stepper = None
        self.fan_name = self._config_get(config, 'fan', None)
        self.fan = None
        if self.fan_name:
            self.toolchanger.require_fan_switcher()
        self.t_command_restore_axis = self._config_get(
            config, 't_command_restore_axis', 'XYZ')
        self.perform_restore_move = self._config_getboolean(
            config, 'perform_restore_move', True)
        self.tool_number = config.getint('tool_number', -1, minval=0)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("ASSIGN_TOOL", "TOOL", self.name,
                                   self.cmd_ASSIGN_TOOL,
                                   desc=self.cmd_ASSIGN_TOOL_help)

        self.printer.register_event_handler("klippy:connect",
                                    self._handle_connect)

    def _handle_connect(self):
        self.extruder = self.printer.lookup_object(
            self.extruder_name) if self.extruder_name else None
        self.extruder_stepper = self.printer.lookup_object(
            self.extruder_stepper_name) if self.extruder_stepper_name else None
        if self.fan_name:
            self.fan = self.printer.lookup_object(self.fan_name,
                      self.printer.lookup_object("fan_generic " + self.fan_name))
        if self.tool_number >= 0:
            self.assign_tool(self.tool_number)
        if getattr(self.detect_mcu, "non_critical_disconnected", False):
            self.is_disconnected = True

    def _handle_detect(self, eventtime, is_triggered):
        _state = self.flip_detect_state ^ is_triggered
        self.detect_state = toolchanger.DETECT_PRESENT if _state else toolchanger.DETECT_ABSENT
        self.toolchanger.note_detect_change(self, eventtime)
        
    # were the cuck here in registering the pin. we know what we want, but we have to check first if someone flipped it.
    # this covers all 4 scenarios automatically, double assigned equal, double assigned equal and not equal, different first, different last.
    def _register_button(self, config, detect_pin_name):
        ppins = self.printer.lookup_object('pins')
        p = ppins.parse_pin(detect_pin_name, can_invert=True, can_pullup=True)
        detect_mcu = p.get('chip', None)
        self.detect_mcu = detect_mcu
        requested_pull = p.get('pullup', 0)
        base = f"{p['chip_name']}:{p['pin']}"
        ppins.allow_multi_use_pin(base)
        prev = ppins.active_pins.get(base)
        # If first, stay "neutral" (noninverted). Otherwise reuse existing polarity.
        if prev is None:
            actual_inv  = 0
            actual_pull = requested_pull
        else:
            actual_inv  = prev.get('invert', 0)
            actual_pull = prev.get('pullup', 0)
            if bool(p.get('invert', 0)) != actual_inv or requested_pull != actual_pull:
                logging.info("Reusing detection pin %s for %s with invert=%s pullup=%s (requested invert=%s pullup=%s)",
                    base, self.name, actual_inv, actual_pull, bool(p.get('invert', 0)), requested_pull)
        dec = ''
        if   actual_pull == 1:  dec += '^'
        elif actual_pull == -1: dec += '~'
        if actual_inv:          dec += '!'
        reg = f"{dec}{base}"
        buttons = self.printer.load_object(config, 'buttons')
        def _btn_handler(eventtime, is_triggered, mcu=detect_mcu):
            if getattr(mcu, "non_critical_disconnected", False):
                # Ignore events while the MCU is offline; state handled via events below
                return
            self._handle_detect(eventtime, is_triggered)

        buttons.register_buttons([reg], _btn_handler)
        self.flip_detect_state = config.getboolean('flip_detect', False) ^ bool(actual_inv) ^ bool(p.get('invert', 0))
        self.detect_state = (toolchanger.DETECT_PRESENT if self.flip_detect_state else toolchanger.DETECT_ABSENT)

        # Track non-critical disconnect/reconnect explicitly
        if detect_mcu and hasattr(detect_mcu, "get_non_critical_disconnect_event_name"):
            def _on_disc():
                self.detect_state = toolchanger.DETECT_UNAVAILABLE
                self.is_disconnected = True
            def _on_recon():
                # Restore baseline state; actual detection follows on next edge
                self.detect_state = (toolchanger.DETECT_PRESENT
                                     if self.flip_detect_state
                                     else toolchanger.DETECT_ABSENT)
                self.is_disconnected = False
            self.printer.register_event_handler(detect_mcu.get_non_critical_disconnect_event_name(), _on_disc)
            self.printer.register_event_handler(detect_mcu.get_non_critical_reconnect_event_name(),  _on_recon)

    def get_status(self, eventtime):
        s = {**self.params,
            'name': self.name,
            'toolchanger': self.toolchanger.name,
            'tool_number': self.tool_number,
            'extruder': self.extruder_name,
            'extruder_stepper': self.extruder_stepper_name,
            'fan': self.fan_name,
            'active': self.main_toolchanger.get_selected_tool() == self,
            'gcode_x_offset': self.gcode_x_offset or 0.0,
            'gcode_y_offset': self.gcode_y_offset or 0.0,
            'gcode_z_offset': self.gcode_z_offset or 0.0,
        }
        if self.detect_mcu and hasattr(self.detect_mcu, "get_non_critical_disconnect_event_name"):
            s['is_disconnected'] = self.is_disconnected
        return s

    def get_offset(self):
        return [self.gcode_x_offset, self.gcode_y_offset, self.gcode_z_offset]

    cmd_ASSIGN_TOOL_help = 'Assign tool to tool number'
    def cmd_ASSIGN_TOOL(self, gcmd):
        self.assign_tool(gcmd.get_int('N', minval=0), replace = True)

    def assign_tool(self, number, replace = False):
        prev_number = self.tool_number
        self.tool_number = number
        self.main_toolchanger.assign_tool(self, number, prev_number, replace)
        self.register_t_gcode(number)

    def register_t_gcode(self, number):
        gcode = self.printer.lookup_object('gcode')
        name = 'T%d' % (number)
        desc = 'Select tool %d' % (number)
        existing = gcode.register_command(name, None)
        if existing:
            # Do not mess with existing
            gcode.register_command(name, existing)
        else:
            tc = self.main_toolchanger
            axis = self.t_command_restore_axis
            func = lambda gcmd: tc.select_tool(
                gcmd, tc.lookup_tool(number), axis)
            gcode.register_command(name, func, desc=desc)

    def activate(self):
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        hotend_extruder = toolhead.get_extruder().name
        if self.extruder_name and self.extruder_name != hotend_extruder:
            gcode.run_script_from_command(
                "ACTIVATE_EXTRUDER EXTRUDER='%s'" % (self.extruder_name,))
        hotend_extruder = toolhead.get_extruder().name
        if self.extruder_stepper and hotend_extruder:
                gcode.run_script_from_command(
                    "SYNC_EXTRUDER_MOTION EXTRUDER='%s' MOTION_QUEUE=" % (self.extruder_stepper_name, ))
                gcode.run_script_from_command(
                    "SYNC_EXTRUDER_MOTION EXTRUDER='%s' MOTION_QUEUE='%s'" % (self.extruder_stepper_name, hotend_extruder, ))
        if self.fan:
            self.toolchanger.fan_switcher.activate_fan(self.fan)
    def deactivate(self):
        if self.extruder_stepper:
            toolhead = self.printer.lookup_object('toolhead')
            gcode = self.printer.lookup_object('gcode')
            hotend_extruder = toolhead.get_extruder().name
            gcode.run_script_from_command(
                "SYNC_EXTRUDER_MOTION EXTRUDER='%s' MOTION_QUEUE=" % (self.extruder_stepper_name,))

    def _config_get(self, config, name, default_value):
        return config.get(name, self.toolchanger.config.get(name, default_value))
    def _config_getfloat(self, config, name, default_value):
        return config.getfloat(name, self.toolchanger.config.getfloat(name, default_value))
    def _config_getboolean(self, config, name, default_value):
        return config.getboolean(name, self.toolchanger.config.getboolean(name, default_value))

def load_config_prefix(config):
    return Tool(config)
