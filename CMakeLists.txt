cmake_minimum_required(VERSION 3.20)

project(ros2_control_py VERSION 0.0.2)

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
add_subdirectory(builder)

# Vars
set(_ros2_py_src_dir ${CMAKE_CURRENT_BINARY_DIR}/src)
set(_ros2_py_src ${_ros2_py_src_dir}/impl_ros2_control_py.cpp)

# calls builder
add_custom_command(
  OUTPUT ${_ros2_py_src}
  COMMAND ${CMAKE_COMMAND} -E env "LSAN_OPTIONS=suppressions=${CMAKE_CURRENT_LIST_DIR}/.lsan.supp" $<TARGET_FILE:builder> "${_ros2_py_src_dir}" "$ENV{AMENT_PREFIX_PATH}" "$ENV{ROS_DISTRO}" ${_ros2_py_modules}
  DEPENDS builder
)
add_custom_target(run_builder
  DEPENDS ${_ros2_py_src}
)

# Create ros2_control_py/ dir in build/
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})
# pybind11 impl
set(_ros2_py_impl _impl_${PROJECT_NAME})
pybind11_add_module(${_ros2_py_impl}
  ${_ros2_py_src}
)
set_target_properties(${_ros2_py_impl}
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME}/${_ros2_py_impl}
)
# sanitize address gcc/clang
if(SANITIZE)
  target_compile_options(${_ros2_py_impl} PRIVATE
    "-fsanitize=undefined"
  )
  target_link_options(${_ros2_py_impl} PRIVATE
    "-fsanitize=undefined"
  )
endif()
add_dependencies(${_ros2_py_impl} run_builder)
target_include_directories(${_ros2_py_impl} PRIVATE
  ${_ros2_py_src_dir}
)
ament_target_dependencies(${_ros2_py_impl} PUBLIC
  rclcpp
  rclcpp_lifecycle
  ${_ros2_py_modules}
)
# __init__.py in build/ for tests
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/python/${PROJECT_NAME}
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
  FILES_MATCHING PATTERN __init__.py
)
# __init__.py in install/
ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR ${CMAKE_CURRENT_LIST_DIR}/python/${PROJECT_NAME}
)
# pybind11 impl in install/
# NOTE: ${PYTHON_INSTALL_DIR} is exported by ament_python_install_package() above
install(TARGETS ${_ros2_py_impl}
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
)

# python tests in tests/
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  file(GLOB_RECURSE _pytest_tests CONFIGURE_DEPENDS RELATIVE ${CMAKE_CURRENT_LIST_DIR} "tests/test_*.py" "tests/*/test_*.py")
  foreach(_test_path ${_pytest_tests})
    cmake_path(RELATIVE_PATH _test_path
      BASE_DIRECTORY tests/
      OUTPUT_VARIABLE _test_name
    )
    ament_add_pytest_test(${_test_name} ${_test_path})
  endforeach()
endif()

ament_package()
