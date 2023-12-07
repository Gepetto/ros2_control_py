# Python bindings for ros2_control using pybind11

[![ros2_control_py humble](https://github.com/Gepetto/ros2_control_py/actions/workflows/humble.yml/badge.svg?branch=main)](https://github.com/Gepetto/ros2_control_py/actions/workflows/humble.yml?query=branch%3Amain)
[![ros2_control_py rolling](https://github.com/Gepetto/ros2_control_py/actions/workflows/rolling.yml/badge.svg?branch=main)](https://github.com/Gepetto/ros2_control_py/actions/workflows/rolling.yml?query=branch%3Amain)

## Install

Assuming that your ros2_control_py workspace is in the directory `~/ros2_control_py_ws`:

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
colcon build --cmake-args ' -DCMAKE_BUILD_TYPE=Debug' ' -DSANITIZE=True'
colcon test
```

## Usage

See [doc](doc/index.rst) and [tests](tests/).
