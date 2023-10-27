# Python bindings for ros2_control using pybind11

-------------------------------------------------

## Install

Assuming that your [ros2_control](https://github.com/ros-controls/ros2_control) workspace is in the directory `~/ros2_control_py_ws`:

```sh
mkdir -p ~/ros2_control_py_ws/src
cd ~/ros2_control_py_ws/src
git clone https://github.com/Gepetto/ros2_control_py --recursive
cd ~/ros2_control_py_ws
rosdep install --from-paths src -y --ignore-src
```

## Compiling

```sh
cd ~/ros2_control_py_ws
colcon build
```

## Testing

```sh
cd ~/ros2_control_py_ws
colcon build --cmake-args ' -DCMAKE_BUILD_TYPE=Debug -DSANITIZE'
colcon test
```

## Usage

See [doc](doc/index.rst) and [tests](tests/).
