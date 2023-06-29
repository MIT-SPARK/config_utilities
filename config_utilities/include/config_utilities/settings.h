#pragma once

namespace config {

namespace internal {

/**
 * @brief Global settings for how config_utilities-based configs behave. These
 * can be dynamically set and changed throughout a program.
 */
struct Settings {
  Settings(const Settings& other) = delete;
  Settings& operator=(const Settings& other) = delete;

  // Singleton access to the global settings.
  static Settings& instance() {
    static Settings settings;
    return settings;
  }

  // Printing Settings.
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

 private:
  Settings() = default;
};

}  // namespace internal

// Access function in regular namespace.

/**
 * @brief Global settings for how config_utilities-based configs behave. These
 * can be dynamically set and changed throughout a program.
 */
internal::Settings& Settings() { return internal::Settings::instance(); }

}  // namespace config