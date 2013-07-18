#include "options/options.h"
#include <iostream>

namespace uxvcos {
namespace Options {

static Description _options;
static Map _map;
static Positional _positional;

Description& options() {
  return _options;
}

options_description_easy_init add_options() {
  return _options.add_options();
}

Map& variables() {
  return _map;
}

Positional& positional() {
  return _positional;
}

bool parse(int argc, char ** argv) {
  try {
    store(command_line_parser(argc, argv).options(_options).positional(_positional).run(), _map);
    notify(_map);
  } catch(error &e) {
    return false;
  }
  return true;
}

void help() {
  std::cout << _options << "\n";
}

} // namespace Options
} // namespace uxvcos
