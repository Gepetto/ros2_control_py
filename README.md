# Python bindings for ros2_control using pybind11
-------------------------------------------------

## Install

### Going to a ROS-2 workspace
Assuming that your [ros2_control](https://github.com/ros-controls/ros2_control) workspace is ìn the directory ```~/ros2_control_ws```:
```
cd ~/ros2_control_ws/src
```

### Getting the source
Currently only two ROS-2 releases are supported: rolling and humble.
The ```main``` branch is for the ROS-2 rolling release and the ```humble``` branch is for the ROS-2 humble release.

For rolling, please types:
```
git clone https://github.com/olivier-stasse/ros2_control_py.git
```

For humble, please type:
```
git clone https://github.com/olivier-stasse/ros2_control_py.git -b humble
```

### Compiling

```
cd ~/ros2_control
colcon build --packages-select ros2_control_py
```

## Testing

To create an actuator
```
from ros2_control_py.core.hardware_interface import Actuator
anActuator = Actuator()
```
