Python bindings for ros2_control using pybind11
================================================================

The goal of this repository is to provide python bindings to the ros2_control C++ framework.
Whereas [ros2_control](https://github.com/ros-controls/ros2_control) shines to deploy and reuse robot control infrastructure on industrial robotics platform
it is quite difficult to prototype controller quickly.

The aim of this repository is to investiguate the possibility of lowering such barrier by providing
python bindings.

The first target is to provide a simple access to the robot hardware through python.
It is based on user experience from several open source libraries facilitating the low level access
through python: [Upkie](https://github.com/upkie/upkie) or [ODRI](https://github.com/open-dynamic-robot-initiative/).

The second target will be to reused and provide a simple way to test various controllers.

# Install
```
mkdir ~/ros2_control_py_ws/src/ros2_control_py
cd ~/ros2_control_py_ws/src/ros2_control_py
git clone git@github.com:Gepetto/ros2_control_py.git
colcon build --packages-up-to ros2_control_python
```

# Test
```
cd ~/ros2_control_py_ws
python ./src/ros2_control_py/ros2_control_py/tests/controller_manager/future_test_control_node.py
```

# Contributors

- Maxence Michot
- Olivier Stasse
