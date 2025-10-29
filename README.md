
## Installation
1. Install Spot library


## Usage
To run without planning shield:
```
ros2 launch aw_runtime_monitor monitor.launch.py output_path:=<your-path>
```

By default, the shield is disabled. The desired path to save trace file should be passed to output_path param.

To run with planning shielding:
```
ros2 launch aw_runtime_monitor monitor.launch.py planning_shield_enabled:=true output_path:=<your-path>
```

