cmake_minimum_required(VERSION 3.5)

project(ros2_control_py)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_C_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
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

set(ros2_py_src_dir ${CMAKE_CURRENT_BINARY_DIR}/src)
set(ros2_py_modules hardware_interface)
set(ros2_py_src ${ros2_py_modules})
list(TRANSFORM ros2_py_src PREPEND ${ros2_py_src_dir}/)
list(TRANSFORM ros2_py_src APPEND _py.cpp)

add_executable(ros2_control_py_builder
  src/ros2_control_py_builder/main.cpp
)
target_link_libraries(ros2_control_py_builder
  PRIVATE
  cppparser
)
add_custom_command(
  OUTPUT ${ros2_py_src}
  COMMAND ros2_control_py_builder "${CMAKE_CURRENT_BINARY_DIR}" "/opt/ros/$ENV{ROS_DISTRO}/include" ${ros2_py_modules}
  DEPENDS ros2_control_py_builder
)

ament_python_install_package(ros2_control_py)

foreach(module ${ros2_py_modules})
  pybind11_add_module(${module}
    ${ros2_py_src}
  )
  ament_target_dependencies(${module} PUBLIC
    rclpy
    rclcpp
    rclcpp_lifecycle
    hardware_interface
  )
  install(TARGETS ${module}
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endforeach()

ament_package()
