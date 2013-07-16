#ifndef OPTIONS_H
#define OPTIONS_H

#include <uxvcos.h>
#include <boost/program_options.hpp>

namespace uxvcos {
namespace Options {
  using namespace boost::program_options;

  typedef options_description Description;
  typedef variables_map Map;
  typedef positional_options_description Positional;

  UXVCOS_API Description& options();
  UXVCOS_API options_description_easy_init add_options();

  UXVCOS_API Map& variables();
  UXVCOS_API Positional& positional();

  UXVCOS_API bool parse(int argc, char ** argv);
  UXVCOS_API void help();

  static inline const variable_value& variables(const std::string &name) {
    return variables()[name];
  }

  template <typename T>
  static inline const T & variables(const std::string &name) {
    return variables()[name].as<T>();
  }

} // namespace Options
} // namespace uxvcos

#endif // OPTIONS_H
