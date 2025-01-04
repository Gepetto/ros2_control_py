Python bindings for ros2_control using pybind11
================================================================

# Introduction

## Motivation
The goal of this repository is to provide python bindings to the ros2_control C++ framework.
Whereas [ros2_control](https://github.com/ros-controls/ros2_control) shines to deploy and reuse robot control infrastructure on industrial robotics platform
it is quite difficult to prototype controller quickly.

The aim of this repository is to investiguate the possibility of lowering such barrier by providing
python bindings.

The first target is to provide a simple access to the robot hardware through python.
It is based on user experience from several open source libraries facilitating the low level access
through python: [Upkie](https://github.com/upkie/upkie) or [ODRI](https://github.com/open-dynamic-robot-initiative/).

The second target will be to provide a simple way to test, resuse various controllers.

## Warning
Python at the control level is often seen as not seriously providing real time capabilities.
And this is probably true if one want to handle an industrial robot working 7 days on 7, 24 hours a day.
However the current Python capabilities provides now a simple way to get 1 KHz and do quick prototyping
simply. Many research teams have successfully applied this approach.
This repository targets mostly this public.

# Install
```
mkdir ~/ros2_control_py_ws/src/ros2_control_py
cd ~/ros2_control_py_ws/src/ros2_control_py
git clone git@github.com:Gepetto/ros2_control_py.git
colcon build --packages-up-to ros2_control_python
```

# Test
The current test is tentative to reimplement the [main control loop ros2_control_node](https://github.com/ros-controls/ros2_control/blob/master/controller_manager/src/ros2_control_node.cpp) in ros2_control.
```
cd ~/ros2_control_py_ws
python ./src/ros2_control_py/ros2_control_py/tests/controller_manager/future_test_control_node.py
```

# Contributors

- Maxence Michot
- Olivier Stasse
