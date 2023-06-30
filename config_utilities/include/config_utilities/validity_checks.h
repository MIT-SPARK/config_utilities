#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/printing_tools.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config {

namespace internal {

// NOTE(lschmid): Currently the formatting of warnings for validity checks is handled here manually. This could well
// also be handed down to a formatting.
std::string formatWarnings(const std::vector<std::string>& warnings, const std::string& name) {
  const std::string sev = "Warning: ";
  const size_t print_width = Settings::instance().print_width;
  const size_t length = print_width - sev.length();
  std::string warning = "Invalid config '" + name + "':\n" + internal::printCenter(name, print_width, '=');
  for (std::string w : warnings) {
    std::string line = sev;
    while (w.length() > length) {
      line.append(w.substr(0, length));
      w = w.substr(length);
      warning.append("\n" + line);
      line = std::string(sev.length(), ' ');
    }
    warning.append("\n" + line + w);
  }
  warning = warning + "\n" + std::string(print_width, '=');
  return warning;
}

}  // namespace internal

/**
 * @brief Check if a config is valid.
 *
 * @tparam ConfigT The config type.
 * @param config The config to check.
 * @param print_warnings Whether to print warnings if the config is not valid.
 * @returns True if the config is valid, false otherwise.
 */
template <typename ConfigT>
bool isValid(const ConfigT& config, bool print_warnings = false) {
  if (!isConfig<ConfigT>()) {
    if (print_warnings) {
      std::stringstream ss;
      ss << "Can not use 'config::isValid()' on non-config T='" << typeid(ConfigT).name()
         << "'. Please implement 'void declare_config(T&)' for your struct.";
      internal::Logger::logWarning(ss.str());
    }
    return false;
  }
  internal::MetaData data = internal::Visitor::getChecks(config);
  if (data.warnings.empty()) {
    return true;
  }
  if (print_warnings) {
    internal::Logger::logWarning(internal::formatWarnings(data.warnings, data.name));
  }
  return false;
}

/**
 * @brief Assert that a config is valid. This will terminate the program if invalid.
 *
 * @tparam ConfigT The config type.
 * @param config The config to check.
 */
template <typename ConfigT>
void checkValid(const ConfigT& config) {
  if (!isConfig<ConfigT>()) {
    std::stringstream ss;
    ss << "Can not use 'config::checkValid()' on non-config T='" << typeid(ConfigT).name()
       << "'. Please implement 'void declare_config(T&)' for your struct.";
    internal::Logger::logFatal(ss.str());
  }
  internal::MetaData data = internal::Visitor::getChecks(config);
  if (data.warnings.empty()) {
    return;
  }
  internal::Logger::logFatal(internal::formatWarnings(data.warnings, data.name));
}

}  // namespace config
