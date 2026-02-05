# Probe multiplexer

# Configuration

Each `[probe_multiplexer <name>]` defines a physical probe and has the same options as `[probe]`

`z_offset` is required for each physical probe.  
`x_offset` and `y_offset` default to 0.

```ini
[stepper_z]
endstop_pin: probe:z_virtual_endstop

[probe_multiplexer T0]
pin: ^PA0
probe_number: 0
x_offset: 0.0
y_offset: 0.0
z_offset: -0.90
speed: 5.0
lift_speed: 10.0
samples: 3
samples_result: median
sample_retract_dist: 2.0
samples_tolerance: 0.02
samples_tolerance_retries: 3
```

> Tip: If you set `probe_number`, you can always select probes by number and
> avoid quoting names in G-code.

# Status

`printer.probe_multiplexer`:

- `probe` - active probe name, or `None`
- `probe_number` - active probe number, or `-1`
- `x_offset`, `y_offset`, `z_offset` - active probe offsets (None if no probe)
- `probe_names` - list of configured probe names
- `probe_numbers` - list of configured probe numbers

# Gcodes

### SET_ACTIVE_PROBE
`SET_ACTIVE_PROBE [P/PROBE=<name>] [N/NUMBER=<number>]`  
Select the active probe by name or number.  
Use `N=-1` or `P=none` to clear the active probe.

> remember to use quotes for names:
> ```
> SET_ACTIVE_PROBE PROBE="probe_multiplexer T0"
> ```

### SET_PROBE_OFFSET
`SET_PROBE_OFFSET [P=<name>] [N=<number>] [X=<x>] [Y=<y>] [Z=<z>]`  
Updates offsets for a probe (defaults to the active probe).

### SAVE_PROBE_OFFSET
`SAVE_PROBE_OFFSET [P=<name>] [N=<number>]`  
Saves offsets for a probe to the config (defaults to the active probe).

### Standard probe commands
All regular Klipper probe commands work (PROBE, QUERY_PROBE, PROBE_CALIBRATE, etc).  
They will error if no active probe is selected.

# Usage

- Call `SET_ACTIVE_PROBE` before any homing or probing move.
- When used with toolchanger, select the probe in your pickup/dropoff macros
  (for example `SET_ACTIVE_PROBE N=<probe_number>` or `SET_ACTIVE_PROBE P=<name>`).
