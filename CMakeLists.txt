cmake_minimum_required(VERSION 3.5)
project(ros2_control_py)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

add_subdirectory(CppParser)

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

add_executable(ros2_control_py_builder
  src/ros2_control_py_builder/main.cpp
)
target_link_libraries(ros2_control_py_builder
  PRIVATE
  cppparser
)
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/src/ros2_control_py/hardware_interface/hardware_interface2.cpp
  COMMAND ros2_control_py_builder ${CMAKE_CURRENT_BINARY_DIR}/src/ros2_control_py/hardware_interface/hardware_interface2.cpp
  DEPENDS ros2_control_py_builder
)

ament_python_install_package(ros2_control_py)

pybind11_add_module(ros2_control_py_core
  src/ros2_control_py/core.cpp
  src/ros2_control_py/hardware_interface/hardware_interface.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/src/ros2_control_py/hardware_interface/hardware_interface2.cpp
)
ament_target_dependencies(ros2_control_py_core
  PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)
target_include_directories(ros2_control_py_core
  PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
install(
  TARGETS ros2_control_py_core
  DESTINATION "${PYTHON_INSTALL_DIR}/ros2_control_py"
)

ament_package()
