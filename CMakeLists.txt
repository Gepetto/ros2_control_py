cmake_minimum_required(VERSION 3.5)
project(ros2_control_py)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

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

add_subdirectory(cppparser)

set(hi_py_cpp ${CMAKE_CURRENT_BINARY_DIR}/src/ros2_control_py/hardware_interface_py.cpp)

add_executable(ros2_control_py_builder
  src/ros2_control_py_builder/main.cpp
)
target_link_libraries(ros2_control_py_builder
  PRIVATE
  cppparser
)
add_custom_command(
  OUTPUT ${hi_py_cpp}
  COMMAND ros2_control_py_builder /opt/ros/$ENV{ROS_DISTRO}/include/hardware_interface ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ros2_control_py_builder
)

ament_python_install_package(ros2_control_py)

pybind11_add_module(hardware_interface_py
  ${hi_py_cpp}
)
ament_target_dependencies(hardware_interface_py
  PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)
install(
  TARGETS hardware_interface_py
  DESTINATION "${PYTHON_INSTALL_DIR}/ros2_control_py"
)

ament_package()
