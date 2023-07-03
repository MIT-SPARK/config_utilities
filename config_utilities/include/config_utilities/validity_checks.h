#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config {

/**
 * @brief Check if a config is valid.
 *
 * @tparam ConfigT The config type.
 * @param config The config to check.
 * @param print_warnings Whether to print warnings if the config is not valid. This is off by default.
 * @param logger [Advanced use] Optionally specify a specific logger for this call. Otherwise uses the global default.
 * @param formatter [Advanced use] Optionally pass a specific formatter for this call. Otherwise uses the global
 * default.
 * @returns True if the config is valid, false otherwise.
 */
template <typename ConfigT>
bool isValid(const ConfigT& config,
             bool print_warnings = false,
             internal::Logger::Ptr logger = internal::Logger::defaultLogger(),
             internal::Formatter::Ptr formatter = internal::Formatter::defaultFormatter()) {
  if (!isConfig<ConfigT>()) {
    if (print_warnings) {
      std::stringstream ss;
      ss << "Can not use 'config::isValid()' on non-config T='" << typeid(ConfigT).name()
         << "'. Please implement 'void declare_config(T&)' for your struct.";
      logger->logWarning(ss.str());
    }
    return false;
  }
  internal::MetaData data = internal::Visitor::getChecks(config);
  if (data.warnings.empty()) {
    return true;
  }
  if (print_warnings) {
    logger->logWarning(formatter->formatCheckWarnings(data));
  }
  return false;
}

/**
 * @brief Assert that a config is valid. This will terminate the program if invalid.
 *
 * @tparam ConfigT The config type.
 * @param logger [Advanced use] Optionally pass a specific logger to log this call to.
 * @param formatter [Advanced use] Optionally pass a specific formatter to format the warnings with.
 * @param config The config to check.
 */
template <typename ConfigT>
void checkValid(const ConfigT& config,
                internal::Logger::Ptr logger = internal::Logger::defaultLogger(),
                internal::Formatter::Ptr formatter = internal::Formatter::defaultFormatter()) {
  if (!isConfig<ConfigT>()) {
    std::stringstream ss;
    ss << "Can not use 'config::checkValid()' on non-config T='" << typeid(ConfigT).name()
       << "'. Please implement 'void declare_config(T&)' for your struct.";
    logger->logFatal(ss.str());
  }
  internal::MetaData data = internal::Visitor::getChecks(config);
  if (data.warnings.empty()) {
    return;
  }
  logger->logFatal(formatter->formatCheckWarnings(data));
}

}  // namespace config
