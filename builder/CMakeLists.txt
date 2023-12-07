cmake_minimum_required(VERSION 3.5)

add_subdirectory(cppparser)

add_executable(builder
  src/main.cpp
)
target_link_libraries(builder
  PRIVATE
  cppparser
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
