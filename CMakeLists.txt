cmake_minimum_required(VERSION 3.20)

project(ros2_control_py VERSION 0.0.2)

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
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

add_subdirectory(cppparser)

set(_ros2_py_src_dir ${CMAKE_CURRENT_BINARY_DIR}/src)

set(_ros2_py_modules
  hardware_interface
  controller_interface
  ros2_control_test_assets
)

set(_ros2_py_src ${_ros2_py_modules})
list(TRANSFORM _ros2_py_src PREPEND ${_ros2_py_src_dir}/)
list(TRANSFORM _ros2_py_src APPEND _py.cpp)

add_executable(${PROJECT_NAME}_builder
  src/${PROJECT_NAME}_builder/main.cpp
)
target_link_libraries(${PROJECT_NAME}_builder
  PRIVATE
  cppparser
)
add_custom_command(
  OUTPUT ${_ros2_py_src}
  COMMAND ${PROJECT_NAME}_builder "${CMAKE_CURRENT_BINARY_DIR}" "/opt/ros/$ENV{ROS_DISTRO}/include" "$ENV{ROS_DISTRO}" ${_ros2_py_modules}
  DEPENDS ${PROJECT_NAME}_builder
)

configure_file("${CMAKE_CURRENT_LIST_DIR}/python/${PROJECT_NAME}/__init__.py"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/__init__.py"
  COPYONLY
)
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR "${CMAKE_CURRENT_LIST_DIR}/python/${PROJECT_NAME}"
)
foreach(_module ${_ros2_py_modules})
  find_package(${_module} REQUIRED)
  pybind11_add_module(${_module}_py
    "${_ros2_py_src_dir}/${_module}_py.cpp"
  )
  set_target_properties(${_module}_py
    PROPERTIES OUTPUT_NAME ${PROJECT_NAME}/${_module}
  )
  target_include_directories(${_module}_py PRIVATE
    "${_ros2_py_src_dir}"
  )
  ament_target_dependencies(${_module}_py PUBLIC
    rclpy
    ${_module}
  )
  install(TARGETS ${_module}_py
    DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB_RECURSE _pytest_tests CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "tests/test_*.py" "tests/*/test_*.py")
  foreach(_test_path ${_pytest_tests})
    cmake_path(RELATIVE_PATH _test_path
      BASE_DIRECTORY "tests/"
      OUTPUT_VARIABLE _test_name
    )
    ament_add_pytest_test(${_test_name} ${_test_path})
  endforeach()
endif()

ament_package()
