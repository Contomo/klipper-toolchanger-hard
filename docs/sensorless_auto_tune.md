# Sensorless Auto Tune

This module performs an autotune by sweeping:

* **StallGuard threshold** (`sgthrs`, `sg4_thrs`, or `sgt`) 
* **Homing current** (temporarily applied during each test point) 

It runs real homing moves, measures how far the axis travels before the stall triggers, and then **stages the best result for `SAVE_CONFIG`** in your `[tmcXXXX stepper_*]` section. 

---

## Scope and requirements

### Supported axes

* **X/Y/Z only** (for now), dual/awd motors untested. 

### Tips and tricks:
- prehome and drive to a known location (50mm for example), run the routine and note down the resulting distance, (50.4mm for example), this will be your new distance to the endstop, which means, that your entire refrence frame shifted by 0.4mm, so for example if you've entered dock locations somewhere, adjust them accordingly by those 0.4mm.

---

## Configuration

Create one section per stepper you want to tune.

### Section name

The section name format is:

```ini
[sensorless_auto_tune stepper_x]
```
### Options

```ini
[sensorless_auto_tune stepper_x]
# Sweep: home current range
# Defaults are derived from the matching [tmc… stepper_x] section:
# - if home_current exists: 0.80×..1.20× home_current (kalico)
# - else: 0.60×..1.00× run_current
home_current_min: 0.70
home_current_max: 1.00
home_current_steps: 6

# Sweep: StallGuard threshold range
# If omitted, defaults to the detected field’s full legal range. 
stall_min: -3 
stall_max:  3 # ← you want to add those

# How many homing runs to perform at each (current, stall) grid point
# note that early failures will not run X times, if a home fails on the first try, that iteration is skipped.
runs_per_test: 6

# How far beyond the currently-known wall distance we allow while validating stability.
# ie the "how much inside the axis are we allowing" (large values would mean skipped steps/ramming into endstop, small values would mean discarding possibly still valid triggers) 
overtravel_window: 0.25

# Speed used when restoring position during internal “preserve position” logic
restore_speed: 100.0
```

---

## Commands

Both commands are registered as **mux commands** keyed by `AXIS`. 
That means you call them like: `… AXIS=X` (or Y/Z).

---

### `SENSORLESS_AUTOTUNE`

Runs the full grid scan and stages the best result for `SAVE_CONFIG`. 
> *(note that for klipper it wont stage current, since it doesnt have `home_current`)*
#### Syntax

```gcode
SENSORLESS_AUTOTUNE AXIS=<X|Y|Z> \
  [OVERTRAVEL_WINDOW=<float>] \
  [RUNS_PER_TEST=<int>] \
  [HOME_CURRENT_MIN=<float>] \
  [HOME_CURRENT_MAX=<float>] \
  [HOME_CURRENT_STEPS=<int>] \
  [STALL_MIN=<int>] \
  [STALL_MAX=<int>]
```

All parameters above are read directly by the command implementation. 

#### What it does (high level)

1. **Preflight sanity check**
   Temporarily sets the StallGuard threshold to “max sensitivity” and performs a homing move limited to **10mm**. If there is no stall trigger within that distance, the command errors out. 

2. **Compute a trigger threshold**
    That distance moves at max sensitivity and some margin on top is used to determine a false from a true trigger, ie below that+margin → early, above → likely an actual home

3. **Grid sweep**
   For each StallGuard value and each current value, it:

   * applies the current temporarily
   * applies the StallGuard value temporarily
   * performs up to `RUNS_PER_TEST` homing runs and records outcomes 

4. **Classify each run**

   * **Overtravel (`O`)**: no trigger occurred within the allowed max distance. 
   * **Early trigger (`X`)**: trigger distance is at/below the computed threshold, *or* below the running wall-distance cap minus slack. 
   * **OK**: a valid moved distance is recorded. 

5. **Pick the best point**
   A point is “valid” only if it has:

   * **no early triggers**
   * **no overtravels**
   * **at least one OK run** 

   Candidates are sorted by:

   * **lowest standard deviation first** (most consistent)
   * then **highest safety margin** (`mean - threshold`) 

6. **Stage results for `SAVE_CONFIG`**

   * Writes `driver_<FIELD>` in the `[tmc… stepper_*]` section, where `<FIELD>` is `SGTHRS`, `SG4_THRS`, or `SGT` depending on the detected driver.
   * If the driver’s current helper supports `home_current`, it also stages `home_current`; otherwise it prints a reminder with the best current value.

#### Output format

For each StallGuard value, it prints a table:

* `X` = early trigger
* `O` = overtravel
* `n/a` = skipped cell (e.g. fewer runs recorded due to early/overtravel breaking) 
 
---

### `SENSORLESS_AUTOTUNE_FIND_CONSTRAINTS`

Homes in both directions from the same start position to estimate total axis span, then stages either `position_min` or `position_max` in the stepper section for `SAVE_CONFIG`. 

Margin is just whats substracted for saving it, ie substract X from the value we discovered.

#### Syntax

```gcode
SENSORLESS_AUTOTUNE_FIND_CONSTRAINTS AXIS=<X|Y|Z> [MARGIN=<float>]
```

`MARGIN` defaults to `0.1` mm. 

#### What it does

* Runs one homing pass in the normal direction and one in reverse, sums the distances to estimate span. 
* Decides whether the configured endstop is near the min side or max side, then stages the opposite bound accordingly:

  * endstop near min → stage `position_max = endstop + span - margin`
  * endstop near max → stage `position_min = endstop - span + margin`
* Writes the chosen value to `[stepper_*]` and prints “Staged for SAVE_CONFIG …”. 

---

## Practical safety notes

* **If the axis is not already homed**, the autotune’s first attempts may allow long travel, because `max_distance` defaults to `1.5 × axis_span` *(default homing)*. 
  Recommendation: home the axis normally before running `SENSORLESS_AUTOTUNE`, so the module starts with an initial wall distance estimate. 


---

## Troubleshooting cues

* **`Precheck failed: no stall within 10.0mm at max sensitivity`**
  The preflight could not trigger a stall even at maximum sensitivity within the 10mm limit. 
  → Check configuration/wirering

* **`no stable region found`**
  Every tested grid point had either early triggers, overtravels, or no usable “OK” samples.

---

## What gets written by `SAVE_CONFIG`

After a successful run, you should expect staged lines in your `[tmc… stepper_*]` section like:

* `driver_SGTHRS` / `driver_SG4_THRS` / `driver_SGT` (depending on detected field)
* optionally `home_current` if supported

And `SENSORLESS_AUTOTUNE_FIND_CONSTRAINTS` will stage either `position_min` or `position_max` in the `[stepper_*]` section. 
