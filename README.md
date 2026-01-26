# klipper-toolchanger-hard

Aimed to be **fully compatible** with the original `klipper-toolchanger`.  
Just offering more features, extras, and options to expand upon.  
Basically, the "assortment of Klipper extensions" from viesturz, with the "assortment of Klipper extensions" from me.

## Kalico status
* toolchanger, tool probes, calibration, rounded path, heater power distributor are all crosscompatible between kalico and klipper
* manual rail is untested and will likely break on kalico, neither did i test the other extras on kalico yet.

## Installation

To install this plugin, run the installation script using the following command over SSH. This script will download this GitHub repository to your RaspberryPi home directory, and symlink the files in the Klipper extra folder.

```commandline
wget -O - https://raw.githubusercontent.com/Contomo/klipper-toolchanger-hard/main/install.sh | bash
```

## Updates that add new files

Note that if an update has new `klipper/extra` files, they **will not** be automatically installed into Klipper.
You will need to run the intall script manualy to add them:

```commandline
bash ~/klipper-toolchanger/install.sh
```

## Main-Components

* [Toolchanger](docs/toolchanger.md) - tool management support.
* [Tool probe](docs/tool_probe.md) - per tool Z probe.
* [Rounded path](docs/rounded_path.md) - rounds the travel path corners for fast non-print moves.
* [Tools calibrate](docs/tools_calibrate.md) - support for contact based XYZ offset calibration probes.

## Extra-Components

* [Heater power distributor](docs/heater_power_distributor.md) - dynamic heater group power limiting (toolheads for example)
* [Sensorless auto tune](docs/sensorless_auto_tune.md) - automatically determines SGT/SGTHR values for your G28
* [Tool drop detection](docs/tool_drop_detection.md) - continous polling of accelerometers, with peak/angle detection (tool dropped)
* [Heater chamber fan](docs/heater_chamber_fan.md) - like a temperature_fan but it heats. automated chamber filter fan using bed as heater.

## Migrating from KTC-Easy to KTC-H

Download & Save a backup of your current configuration.   

Uninstall KTC-E and install KTC-H.  
```commandline
sudo rm -r ~/klipper-toolchanger-easy
```
```commandline
wget -O - https://raw.githubusercontent.com/Contomo/klipper-toolchanger-hard/main/install.sh | bash
```


In Mainsail, delete everything in the toolchanger folder. Don't delete any Tool configs and toolchanger-config.cfg if those happen to be in here.   
Restore the readonly-configs from the KTC-E GitHub by putting them in the toolchanger folder.   
You can find them here:
```commandline
https://github.com/jwellman80/klipper-toolchanger-easy/tree/main/examples/easy-additions
```
from /user-configs, ONLY download toolchanger-include.cfg for TAP setups. toolchanger-include_scanner.cfg for scanners. 
```commandline
https://github.com/jwellman80/klipper-toolchanger-easy/tree/main/examples/easy-additions/user-configs
```

Set up your files like so:
```commandline
├── toolchanger/
│   ├── homing.cfg				 	 # from easy-additions folder
│   ├── tool_detection.cfg     	 # for TAP install
│   ├── toolchanger.cfg
│   ├── toolchanger-macros.cfg
│   ├── calibrate-offsets.cfg
│   ├── crash-detection.cfg
│   └── toolchanger-include.cfg
├── tools/                   	 # Tool-specific configurations
│   ├── T0.cfg          			 # More tools as needed
│   └── T1.cfg
└── toolchanger-config.cfg		 # User-editable overrides
└── printer.cfg      				 # your other CFGs are also here 
```


Edit the toolchanger-include.cfg to include your tools, for example: 
```commandline
[include ../tools/T0.cfg]
[include ../tools/T1.cfg]
```

Now your toolchanger-include.cfg will include all of the .cfgs :)


Edit your include in printer.cfg:  
FROM: 
```commandline
[include toolchanger/readonly-configs/toolchanger-include.cfg]
```
TO:   
```commandline
[include toolchanger/toolchanger-include.cfg]
```

In tool_detection.cfg, comment out the macros INITIALIZE_TOOLCHANGER, _INITIALIZE_FROM_DETECTED_TOOL, _INITIALIZE_FROM_DETECTED_TOOL_IMPL, VERIFY_TOOL_DETECTED.  

All done! :)
