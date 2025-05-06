cmake_minimum_required(VERSION 3.5)

add_subdirectory(cppast)
add_subdirectory(cppparser)
add_subdirectory(cppwriter)

set(CMAKE_VERBOSE_MAKEFILE ON)
add_executable(builder
  src/main.cpp
)
target_include_directories(builder
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cppparser/include/cppparser>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cppast/include/cppast>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cppwriter/include/cppwriter>
)
target_link_libraries(builder
  PRIVATE
  cppparser
  cppwriter
)
# sanitize address gcc/clang
if(SANITIZE)
  target_compile_options(builder PRIVATE
    "-fno-omit-frame-pointer"
    "-fsanitize=address,undefined,leak"
  )
  target_link_options(builder PRIVATE
    "-fno-omit-frame-pointer"
    "-fsanitize=address,undefined,leak"
  )
endif()

install(TARGETS
  builder
  DESTINATION
  lib/${PROJECT_NAME})
