# toolchanger
right now only the "fixed" dock location is up to date, the others will work but i have not taken the time to adjust/optimize them yet.


# DOCK_TUNER
its an alternative to the tool align provided at the bottom of the toolchanger.cfg  
  
<img src="../../../media/dock_tuner.png" alt="Dock tuner" height="500">

## How do i use this?
- make sure printer is homed
- call `DOCK_TUNER` in console
- do the thing
- leave dock in a "safe_y" position on exit
- select another tool, or leave DOCK_TUNER with no tool attatched and call `DOCK_TUNER T=<your tool number>` for the next one
- remeber to hit or call `SAVE_CONFIG` once done

## Issues and fixes:
- **I dont have params_pickup_path and params_dropoff_path**
change this to whatever you are using *(example `params_sc_path `)* :
```py
{% set pick     = tool.params_pickup_path %}
{% set drop     = tool.params_dropoff_path %}
# >>>  
{% set pick     = tool.params_sc_path %}
{% set drop     = tool.params_sc_path|reverse %}
```

- **`SAVE_CONFIG` telling you to keep yourself safe?**
that usually means your [tool] `params_park_<xyz>`  are not in printer.cfg, it thus can't comment them out.
that means, you have to comment them out yourself first, and then hit `SAVE_CONFIG`
