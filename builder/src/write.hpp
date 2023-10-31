#pragma once

// ros2_control_py_builder
#include "structs.hpp"

/// @brief write source files for modules `mods`
inline void write_impl(const fs::path& src_dir,
                       const std::vector<std::shared_ptr<Module>>& mods,
                       const StlBinderHeader& stl_binder);

// hxx
#include "write.hxx"
