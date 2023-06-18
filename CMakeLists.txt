cmake_minimum_required(VERSION 3.5)
project(ros2_control_py)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
#find_package(controller_interface REQUIRED)
#find_package(controller_manager REQUIRED)
#find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

ament_python_install_package(ros2_control_py)

pybind11_add_module(hardware_interface_py
  src/ros2_control_py/hardware_interface_py.cpp
  src/ros2_control_py/hardware_interface/actuator_interface_py.cpp
  src/ros2_control_py/hardware_interface/actuator_py.cpp
  src/ros2_control_py/hardware_interface/hardware_component_info_py.cpp
  src/ros2_control_py/hardware_interface/hardware_info_py.cpp
  src/ros2_control_py/hardware_interface/sensor_interface_py.cpp
  src/ros2_control_py/hardware_interface/sensor_py.cpp
  src/ros2_control_py/hardware_interface/system_interface_py.cpp
  src/ros2_control_py/hardware_interface/system_py.cpp
)

ament_target_dependencies(hardware_interface_py PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)

install(TARGETS hardware_interface_py
  DESTINATION "${PYTHON_INSTALL_DIR}/ros2_control_py")
ament_package()
