#include <cppparser.h>

#include <filesystem>
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
  namespace fs = std::filesystem;
  if (argc == 2) {
    std::cerr << "ros2_control_py_builder: " << argv[1] << std::endl;
    fs::path file{argv[1]};
    fs::create_directories(file.parent_path());
    std::ofstream ofs{argv[1], std::ios::out | std::ios::trunc};
    if (!ofs) std::cerr << "error!" << std::endl;
    ofs << std::endl;
  }
  std::cerr << "ros2_control_py_builder has ran!" << std::endl;
  return 0;
}
