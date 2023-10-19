#pragma once

// ros2_control_py_builder
#include "structs.hpp"
#include "utils.hpp"

inline void parse_header(Module& mod, fs::path path, const std::string& name);

// hxx
#include "parse.hxx"
