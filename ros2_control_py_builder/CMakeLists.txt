cmake_minimum_required(VERSION 3.20)

project(ros2_control_py_builder VERSION 0.0.2)

## default ros std versions/warnings
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
# Warnings for gcc/clang
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ros2_control modules to parse
set(_ros2_py_modules
  controller_interface
  controller_manager
  hardware_interface
  joint_limits
  ros2_control_test_assets
  transmission_interface
)

# ROS deps
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# Python deps
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
# ros2_control deps
foreach(_module ${_ros2_py_modules})
  find_package(${_module} REQUIRED)
endforeach()

# compiles builder
# The build reads the header files are generates the pybind files
add_subdirectory(builder)

# Vars
# _ros2_py_src_dir contains the directory where the cpp files are generated.
set(_ros2_py_src_dir ${CMAKE_SOURCE_DIR}/../ros2_control_python/src)
set(_ros2_py_src ${_ros2_py_src_dir}/impl_ros2_control_py.cpp)

# calls builder
# with cmake command.
add_custom_command(
  OUTPUT ${_ros2_py_src}
  COMMAND ${CMAKE_COMMAND} -E env "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_LIST_DIR}/.lsan.supp" $<TARGET_FILE:builder> "${_ros2_py_src_dir}" "$ENV{AMENT_PREFIX_PATH}" "$ENV{ROS_DISTRO}" ${_ros2_py_modules}
  DEPENDS builder
)

ament_package()
