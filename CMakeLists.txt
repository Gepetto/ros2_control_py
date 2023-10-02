cmake_minimum_required(VERSION 3.5)
project(ros2_control_py)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(NOT WIN32)
  add_compile_options(-ggdb3 -O0)
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

add_subdirectory(cppparser)

set(hi_py_cpp ${CMAKE_CURRENT_BINARY_DIR}/src/hardware_interface_py.cpp)

add_executable(ros2_control_py_builder
  src/ros2_control_py_builder/main.cpp
)
target_link_libraries(ros2_control_py_builder
  PRIVATE
  cppparser
)
add_custom_command(
  OUTPUT ${hi_py_cpp}
  COMMAND ros2_control_py_builder /opt/ros/humble/include/hardware_interface ${CMAKE_CURRENT_BINARY_DIR}
  DEPENDS ros2_control_py_builder
)

ament_python_install_package(ros2_control_py)

pybind11_add_module(ros2_control_py_core
  ${hi_py_cpp}
)
ament_target_dependencies(ros2_control_py_core
  PUBLIC
  rclcpp
  rclcpp_lifecycle
  hardware_interface
)
target_include_directories(ros2_control_py_core
  PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/include
)
install(
  TARGETS ros2_control_py_core
  DESTINATION "${PYTHON_INSTALL_DIR}/ros2_control_py"
)

ament_package()
