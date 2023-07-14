#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "config_utilities/globals.h"
#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/printing.h"
#include "config_utilities/traits.h"

namespace config {

/**
 * @brief Check if a config is valid.
 *
 * @tparam ConfigT The config type.
 * @param config The config to check.
 * @param print_warnings Whether to print warnings if the config is not valid. This is off by default.
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
  if (!data.hasErrors()) {
    return true;
  }
  if (print_warnings) {
    internal::Logger::logWarning(
        internal::Formatter::formatErrors(data, "Invalid config", internal::Formatter::Severity::kWarning));
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
const ConfigT& checkValid(const ConfigT& config) {
  if (!isConfig<ConfigT>()) {
    std::stringstream ss;
    ss << "Can not use 'config::checkValid()' on non-config T='" << typeid(ConfigT).name()
       << "'. Please implement 'void declare_config(T&)' for your struct.";
    internal::Logger::logFatal(ss.str());
  }
  internal::MetaData data = internal::Visitor::getChecks(config);
  if (!data.hasErrors()) {
    return config;
  }
  internal::Logger::logFatal(
      internal::Formatter::formatErrors(data, "Invalid config", internal::Formatter::Severity::kFatal));
  internal::Globals::instance().valid_configs.emplace_back(toString(config));
  return config;
}

}  // namespace config
