#pragma once

#include <iostream>
#include <fstream>
#include <wordexp.h>
#include <boost/program_options.hpp>
#include <string>

struct prog_options
{
  bool show_help = false;
  bool show_version = false;
  std::string mode;
  std::string output_mode;
  std::string i2c_bus_name;
};

void print_command_line_options_desc();

prog_options get_prog_options(int argc, char ** argv);

