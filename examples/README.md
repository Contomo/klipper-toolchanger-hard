# Toolchanger Config Examples

The examples here are for specific toolchanger setups, and will likely need to be tweaked for your specific case.  
They are all for setups with multiple physical tools.  
The extension code supports arbitrary tool setups, but this has been the most prolific use by far.

## Important note

For kalico and future klipper usage, i plan on deprecating tool_probe_endstop, the general plan is that toolchanger handles tool detection, this makes configs easier, setups cleaner, and the divergency between setups exponentially smaller.


## Klipper Setup 

Each setup is a combination of 3 parts:  
- [Z Probing](#z-probing)
    - [Shuttle Mounted Regular Probe](#shuttle-mounted-regular-probe)
    - [Probe on Tool 0](#probe-on-tool-0)
    - [Probe on All Tools](#probe-on-all-tools)
- [Dock Location](#dock-location)
    - [Fixed Dock](#fixed-dock)
    - [Liftbar](#liftbar)
    - [Liftbar + Flying Gantry](#liftbar--flying-gantry)
- [Tool Mounting System](#tool-mounting-system)
    - [TapChanger](#tapchanger)
    - [StealthChanger](#stealthchanger)
    - [ClickChanger](#clickchanger)

---

## Z Probing

- ### Shuttle Mounted Regular Probe
  *TODO: need an example for this.*  
  **Shuttle mounted cartographer probe** or other eddy current probe.  
  Compatible with all printer types, but requires wiring routed to the shuttle in addition to each tool.  
  Is a good combination together with a CPAP cooling also permanently mounted on the shuttle.  
  Each toolhead has a basic switch or TAP (optical sensor) to detect if the tool is mounted.

- ### Probe on Tool 0
  
  This is the next simplest setup — with a bed probe on T0 for homing and probing.  
  Each toolhead has a basic switch to detect if the tool is mounted.
  
  The main limitation is that if Z homing is needed before tool change, and T0 is not mounted, the homing will fail.  
  
  Suitable for flying gantry printers like Voron 2.4, but might be also adapted for fixed-gantry 
  systems that do not need Z movement for tool change.

- ### Probe on All Tools

  Config where each tool has a separate Z probe, used for both tool detection and homing/probing.  
  This is more versatile but can result in more calibration as you now need to calibrate all tool probes (or do it automatically).
  (note that not all tool probes need to be calibrated if you dont intend to home will all tool probes :P)
  Suitable for flying or fixed gantry printers like Voron 2.4.

---

## Dock Location

- ### Fixed Dock
  
  Simplest mechanical design, but means that the dock stays in the print area, potentially limiting available area.

- ### Liftbar
  
  A system where tool-change Z movement is handled by a separate lifter rail.  
  Suitable for fixed gantry printers, like Voron Trident.

- ### Liftbar + Flying Gantry

  A system where tools are lowered via a separate lifter rail, but the tool change itself
  is handled by only moving the toolhead.


---


## Tool Mounting System

Select a mounting path, depending on your tool mounting system.  
Notable examples would be:

- ### TapChanger
  ```hcl
  params_dropoff_path: [{'z':0, 'y':4}, {'z':0, 'y':0}, {'z':-7.3, 'y':0}, {'z':-11.2, 'y':3.5}, {'z':-13.2, 'y':8}]
  params_pickup_path: [{'z':-13.2, 'y':8}, {'z':-11.2, 'y':3.5}, {'z':-7.3, 'y':0}, {'z':3, 'y':0, 'f':0.5, 'verify':1}, {'z':0, 'y':0}, {'z':0, 'y':4}]
  ````

- ### StealthChanger

  ```hcl
  params_dropoff_path: [{'z':3.5, 'y':4}, {'z':0, 'y':0}, {'z':-12, 'y':0}]
  params_pickup_path: [{'z':-12, 'y':2}, {'z':-12, 'y':0}, {'z':1.5, 'y':0, 'f':0.5, 'verify':1}, {'z':0.5, 'y':2.5, 'f':0.5}, {'z':8, 'y':8}]
  ```

- ### ClickChanger

  ```hcl
  params_dropoff_path: [{'z':0, 'y':10}, {'z':0, 'y':0}, {'z':-8, 'y':0}, {'z':-9, 'y':3}]
  params_pickup_path: [{'z':-9, 'y':3}, {'z':-8, 'y':0}, {'z':-4, 'y':0}, {'z':1, 'f':0.5, 'verify':1}, {'z':0}, {'y':10, 'z':0}]
  ```
  
---

## Slicer Setup

Gcodes for Orca:

- **Start:**

  ```
  PRINT_START TOOL_TEMP={first_layer_temperature[initial_tool]} BED_TEMP=[first_layer_bed_temperature] TOOL=[initial_tool]{if is_extruder_used[0]} T0_TEMP={first_layer_temperature[0]}{endif}{if is_extruder_used[1]} T1_TEMP={first_layer_temperature[1]}{endif}{if is_extruder_used[2]} T2_TEMP={first_layer_temperature[2]}{endif}{if is_extruder_used[3]} T3_TEMP={first_layer_temperature[3]}{endif}{if is_extruder_used[4]} T4_TEMP={first_layer_temperature[4]}{endif}{if is_extruder_used[5]} T5_TEMP={first_layer_temperature[5]}{endif} CHAMBER=[chamber_temperature] EXTRUDER={first_layer_temperature[initial_tool]}
  ```

- **Before layer change:**

  ```
  ;BEFORE_LAYER_CHANGE
  ;[layer_z]
  G92 E0
  ```

- **After layer change:**

  ```
  ;AFTER_LAYER_CHANGE
  ;[layer_z]
  VERIFY_TOOL_DETECTED ASYNC=1 ; Check after each layer if the tool is still attached
  SET_PRINT_STATS_INFO TOTAL_LAYER=[total_layer_count] CURRENT_LAYER=[layer_num]
  ```
