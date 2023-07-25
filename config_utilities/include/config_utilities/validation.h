#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "config_utilities/globals.h"
#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config {

namespace internal {

bool hasNoInvalidChecks(const MetaData& data);

}  // namespace internal

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
  static_assert(isConfig<ConfigT>(),
                "Can not use 'config::isValid()' on non-config type. Please implement 'void declare_config(ConfigT&)' "
                "for your struct.");
  internal::MetaData data = internal::Visitor::getChecks(config);

  if (internal::hasNoInvalidChecks(data)) {
    return true;
  }
  if (print_warnings) {
    internal::Logger::logWarning(
        internal::Formatter::formatErrors(data, "Invalid config", internal::Severity::kWarning));
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
  static_assert(
      isConfig<ConfigT>(),
      "Can not use 'config::checkValid()' on non-config type. Please implement 'void declare_config(ConfigT&)' "
      "for your struct.");
  internal::MetaData data = internal::Visitor::getChecks(config);

  // Write the config data to global storage for later summarization if requested.
  if (internal::hasNoInvalidChecks(data)) {
    if (Settings().store_valid_configs) {
      internal::Globals::instance().valid_configs.emplace_back(internal::Visitor::getValues(config));
    }

    return config;
  }
  internal::Logger::logFatal(internal::Formatter::formatErrors(data, "Invalid config", internal::Severity::kFatal));
  return config;
}

}  // namespace config
