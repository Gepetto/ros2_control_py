cmake_minimum_required(VERSION 3.5)

add_subdirectory(cppparser)

add_executable(builder
  src/main.cpp
)
target_link_libraries(builder
  PRIVATE
  cppparser
)
