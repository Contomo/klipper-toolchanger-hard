
# klipper-toolchanger

Toolchanging extension for Klipper.  
Provides the basic structure and connects tool changes into G-code.  

It is **not opinionated** about how the physical change happens and works for both physical tools and MMUs.  
All actual tool-change motions are implemented by your own G-code macros.

---

## Status

- [x] Single toolchanger works well
- [ ] Cascading tools support (Multiple toolchangers with arbitrary names)

---

## Lifecycle

This is the overall lifecycle of the toolchanger.

Initialization can be done manually from `PRINT_START` or automatically on home or on the first tool change.

![Lifecycle](../media/lifecycle.png)

---

## Config

### `[toolchanger]`

Configures common tool-changing parameters.
> *~~More than one toolchanger can be configured, with arbitrary names.~~(TODO)*

Any parameter that can be set on a tool can also be set here and becomes the default for all of its tools.

```ini
[toolchanger]
initialize_on: <manual|home|first-use>    # default: first-use
initialize_gcode: <template>
verify_tool_pickup: <True|False>          # default: True
require_tool_present: <True|False>        # default: False
uses_axis: <xyz>                          # default: xyz
on_axis_not_homed: <abort|home>           # default: abort
perform_restore_move: <True|False>        # default: True
pickup_gcode: <template>
dropoff_gcode: <template>
before_change_gcode: <template>
after_change_gcode: <template>
error_gcode: <template>                   # required for VERIFY_TOOL_DETECTED ASYNC=1
recover_gcode: <template>                 # used after error_gcode (see INITIALIZE_TOOLCHANGER RECOVER=1)
on_tool_mounted_gcode: <template>         # fires on detection changes (not during toolchanges/probing)
on_tool_removed_gcode: <template>
transfer_fan_speed: <True|False>          # default: True
params_*: <python literal>                # exposed in templates and inherited by tools
```

> *~~The unnamed toolchanger is always considered the main one and others can be connected to a tool in the main toolchanger.~~(TODO)*

### `[tool]`

Defines a tool that can be selected.
A tool usually has an assigned extruder, fans, and related printer config (like pressure advance), but it can be virtual (e.g., a slot in an MMU).
See [Gcodes](#gcodes) for control. Tool params, G-code offsets, and restore-axis behavior inherit from `[toolchanger]` and can be overridden per tool.

```ini
[tool <name>]
extruder: <extruder>
extruder_stepper: <extruder_stepper>      # optional extra stepper to sync via SYNC_EXTRUDER_MOTION
fan: <fan_generic>
detection_pin: <pin>                      # required if any tool uses detection; enables mount/remove callbacks
tool_number: <n>                          # registers T<n>
gcode_x_offset: <mm>
gcode_y_offset: <mm>
gcode_z_offset: <mm>
# Most templates from [toolchanger] can be overridden here.
params_*: <python literal>                # overrides inherited parameters
```

Tool G-code offsets are applied independently from user G-code offsets.
User G-code offsets are preserved across tool changes.

---

## Templates

### Available in all templates

* `tool` — currently active tool object or `None`. This is the actual object (e.g. you can access `tool.fan`).
* `toolchanger` — same as `printer.toolchanger`.

### Available in `before_change_gcode`, `pickup_gcode`, `dropoff_gcode`, `after_change_gcode`

* `dropoff_tool` — name of the tool being dropped off, or `None` if not a dropoff.
* `pickup_tool` — name of the tool being picked up, or `None` if not a pickup.

### `pickup_gcode` and `dropoff_gcode` specifics

* `restore_position.X|Y|Z` — coordinates to restore after the tool change. Only axes requested will be moved. Adjusted for tool offsets.
* `start_position.X|Y|Z` — coordinates before the tool change. Adjusted for tool offsets.

### `on_tool_mounted_gcode` and `on_tool_removed_gcode` specifics

* `detected_tool` — tool object just mounted (or `None` if not a mount).
* `removed_tool` — tool object just removed (or `None` if not a remove).

---

## Gcodes

> [!NOTE]
> To avoid repetition: `[TOOL_NAME=<name>]` and `[T=<number>]` are interchangeable. Only `T=<n>` is shown below, but both are valid.
> Parameters in `[]` are optional. If `T` is optional and omitted, the command applies to the current tool.

The following commands are available when the toolchanger is loaded.

### `INITIALIZE_TOOLCHANGER`

```
INITIALIZE_TOOLCHANGER [TOOLCHANGER=<name>] [T=<n>] [RECOVER=0]
```

Initializes or re-initializes the toolchanger state. Sets status to `ready`.

* By default the toolchanger auto-initializes on the first tool selection.
* Always manually re-initialize after a `SELECT_TOOL_ERROR`.
* If `TOOL_NAME`/`T` is specified, sets the active tool **without** performing tool-change G-code. `after_change_gcode` still runs.
* `TOOL_NAME` with an empty name unselects the tool.
* If left in an `error` state and `RECOVER=1` is specified, `recover_gcode` runs and the toolhead is moved to the restore position similarly to `pickup_gcode`.

### `ASSIGN_TOOL`

```
ASSIGN_TOOL T=<current_n> N=<new_number>
```

Assigns a tool to a specific tool number.
Overrides any assignment from `tool.tool_number`. Registers corresponding `T<n>` and `M106/M107 T<index>` commands.
Does **not** change the active tool.

> [!NOTE]
>  not commonly used; open to concrete use cases.

### `SELECT_TOOL`

```
SELECT_TOOL T=<n> [RESTORE_AXIS=xyz] [X=<float>] [Y=<float>] [Z=<float>]
```

Selects the active tool.

* The toolhead is restored to the previous position on any axis specified in `RESTORE_AXIS`.
* Alternatively, providing `X|Y|Z` overrides the restore position (and marks those axes for restore).
* Slicers typically issue `T0`, `T1`, `T2`, etc. Your config should map these to tool names, or use `tool_number` to auto-register.

Selection sequence:

1. Save G-code state
2. Run `toolchanger.before_change_gcode`
3. Deactivate current extruder/fan if changing
4. Run `current_tool.dropoff_gcode` (if a tool is selected)
5. Run `new_tool.pickup_gcode`
6. Activate new extruder/fan if changing
7. Run `toolchanger.after_change_gcode`
8. Restore G-code state (no move)
9. Move to G-code position according to `RESTORE_AXIS` and any `X|Y|Z` overrides

> *~~If tools have parents, their corresponding dropoff/pickup G-code also runs.~~ (TODO)*

### `SELECT_TOOL_ERROR`

```
SELECT_TOOL_ERROR [MESSAGE="<text>"]
```

Signals failure to select a tool.
May be called from within tool macros during `SELECT_TOOL`. Aborts remaining steps and puts the toolchanger in `ERROR` state, then runs `error_gcode` if provided.

### `UNSELECT_TOOL`

```
UNSELECT_TOOL [RESTORE_AXIS=xyz] [X=<float>] [Y=<float>] [Z=<float>]
```

Unselects the active tool without selecting a new one.
Performs only the first part of `SELECT_TOOL`, leaving the printer with no tool selected.

### `VERIFY_TOOL_DETECTED`

```
VERIFY_TOOL_DETECTED T=<n> [ASYNC=0] [WINDOW=0.005]
```

Verifies that the detected tool matches the expected one. Runs `error_gcode` or shuts down Klipper on failure.
No effect if `detection_pin` is not configured.

* `ASYNC=0`: checks continuously until queued moves are complete. For fastest detection, call as late as you can safely allow.
* `ASYNC=1`: returns immediately and checks in the background. Call at the point you expect the detection pin to have just triggered. On failure, toolchange and restore moves may finish before `error_gcode` executes (all buffers must drain, usually \~1–2 s). Increase `WINDOW` to mitigate false triggers; keep it low for faster fault detection.

On verification failure:

* Abort in-progress tool change
* Enter `ERROR` state
* Run `error_gcode` if provided

### `SET_TOOL_TEMPERATURE`

```
SET_TOOL_TEMPERATURE [T=<n>] TARGET=<temp> [WAIT=0]
```

Sets tool temperature.

### `ENTER_DOCKING_MODE`

```
ENTER_DOCKING_MODE
```

Manually enters docking mode, clearing tool and G-code offsets. Primarily for dock alignment.

### `EXIT_DOCKING_MODE`

```
EXIT_DOCKING_MODE
```

Exits manual docking mode.

### `TEST_TOOL_DOCKING`

```
TEST_TOOL_DOCKING
```

Docks and undocks the current tool. Requires docking mode.

### `SET_TOOL_PARAMETER`

```
SET_TOOL_PARAMETER [T=<n>] PARAMETER=parameter_<name> VALUE=<literal>
```

Changes a tool parameter at runtime.

### `SAVE_TOOL_PARAMETER`

```
SAVE_TOOL_PARAMETER [T=<n>] PARAMETER=parameter_<name>
```

Saves the tool parameter to pending config changes.

### `RESET_TOOL_PARAMETER`

```
RESET_TOOL_PARAMETER [T=<n>] PARAMETER=parameter_<name> VALUE=<literal>
```

Resets a parameter to its original value.

### `SET_TOOL_OFFSET`

```
SET_TOOL_OFFSET [T=<n>] [X=<float>] [Y=<float>] [Z=<float>]
```

Like `SET_TOOL_PARAMETER`, but for G-code offsets.

### `SAVE_TOOL_OFFSET`

```
SAVE_TOOL_OFFSET [T=<n>] [X=<float>] [Y=<float>] [Z=<float>]
```

Like `SAVE_TOOL_PARAMETER`, but for G-code offsets.
If no axis is specified, saves the currently active tool’s offsets.

---

## Gcodes if a *fan* is specified on any tool

### `M106`

```
M106 S<speed> [P<tool number>] [T<tool number>] [TOOL=<tool name>]
```

Sets fan speed. With no `P/T/TOOL`, sets the current tool’s fan.
If `toolchanger.transfer_fan_speed` is enabled, the current fan speed is transferred to the new tool on tool change.

### `M107`

```
M107 [P<tool number>] [T<tool number>] [TOOL=<tool name>]
```

Stops a fan. With no parameters, stops the current tool’s fan.

---

## Status

### `tool` object

* `name` — tool name (e.g., `tool T0`)
* `tool_number` — assigned tool number, or `-1` if unassigned
* `toolchanger` — name of the toolchanger this tool is attached to
* `extruder` — name of the extruder used by this tool
* `fan` — full name of the part-cooling fan for this tool
* `active` — whether this tool is currently selected
* ~~`mounted_child` — child tool currently mounted, or empty~~
* `params_*` — values specified via `params_*`
* `gcode_x_offset` — current X offset
* `gcode_y_offset` — current Y offset
* `gcode_z_offset` — current Z offset

### `toolchanger` object

* `status` — one of `uninitialized`, `ready`, `changing`, `error`
* `tool` — name of the currently selected/changed tool, or empty
* `tool_number` — number of the currently selected tool, or `-1`
* `has_detection` — `True`/`False` 
* `detected_tool` — name of the currently detected tool, or empty
* `detected_tool_number` — number of the currently detected tool, or `-1`
* `tool_numbers` — list of assigned tool numbers, e.g. `[0, 1, 2]`
* `tool_names` — list of tool names corresponding to the assigned numbers
