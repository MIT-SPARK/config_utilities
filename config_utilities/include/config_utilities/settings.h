#pragma once

#include <string>

namespace config {

namespace internal {

/**
 * @brief Global settings for how config_utilities-based configs behave. These
 * can be dynamically set and changed throughout a program.
 */
struct Settings {
  Settings(const Settings& other) = delete;
  Settings(const Settings&& other) = delete;
  Settings& operator=(const Settings& other) = delete;
  Settings& operator=(const Settings&& other) = delete;

  // Singleton access to the global settings.
  static Settings& instance() {
    static Settings settings;
    return settings;
  }

  // Printing Settings. TODO(lschmid): These should probabl be moved to a file or so for different formatters.
  // Width of the 'toString()' output of configs.
  unsigned int print_width = 80u;

  // Indent after which values are printed.
  unsigned int print_indent = 30u;

  // Indent for nested configs.
  unsigned int subconfig_indent = 3u;

  // If true, indicate which values are identical to the default.
  bool indicate_default_values = true;

  // If true, also display the unit of each parameter where provided.
  bool indicate_units = true;

  // If true integrate subconfig fields into the main config, if false print them separately.
  bool index_subconfig_field_names = true;

  // Specify the default logger to be used for printing. Loggers register themselves if included.
  void setLogger(const std::string& name);

  // Specify the formatter to be used for printing. Formatters register themselves if included.
  void setFormatter(const std::string& name);

 private:
  Settings() = default;
};

}  // namespace internal

// Access function in regular namespace.

/**
 * @brief Global settings for how config_utilities-based configs behave. These
 * can be dynamically set and changed throughout a program.
 */
inline internal::Settings& Settings() { return internal::Settings::instance(); }

}  // namespace config
