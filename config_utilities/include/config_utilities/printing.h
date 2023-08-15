#pragma once

#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config {

/**
 * @brief Returns a string representation of the config.
 *
 * @tparam ConfigT The type of the config to print. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param config The config to print. T
 * @param print_warnings If true, prints warnings for any failed conversions.
 * @returns The string representation of the config.
 */
template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type = true>
std::string toString(const ConfigT& config, bool print_warnings = true) {
  // Get the data of the config.
  internal::MetaData data = internal::Visitor::getValues(config);
  // Format the output data.
  if (print_warnings && data.hasErrors()) {
    internal::Logger::logWarning(
        internal::Formatter::formatErrors(data, "Errors parsing config", internal::Severity::kWarning));
  }

  return internal::Formatter::formatConfig(data);
}

/**
 * @brief Returns a string representation of the config.
 *
 * @tparam ConfigT The type of the config to print. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param config The config to print. T
 * @param print_warnings If true, prints warnings for any failed conversions.
 * @returns The string representation of the config.
 */
template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type = true>
std::string toString(const std::vector<ConfigT>& array_config, bool print_warnings = true) {
  // Get the data of the config.
  internal::MetaData data;
  data.name = "Config Array";
  for (size_t i = 0; i < array_config.size(); ++i) {
    auto& d = data.sub_configs.emplace_back(internal::Visitor::getValues(array_config[i]));
    d.array_config_index = i;
    d.field_name = "config_array";
  }
  // Format the output data.
  if (print_warnings && data.hasErrors()) {
    internal::Logger::logWarning(
        internal::Formatter::formatErrors(data, "Errors parsing config", internal::Severity::kWarning));
  }

  return internal::Formatter::formatConfig(data);
}

}  // namespace config

// Define the ostream operator for declared configs.
template <typename ConfigT, typename std::enable_if<config::isConfig<ConfigT>(), bool>::type = true>
std::ostream& operator<<(std::ostream& os, const ConfigT& config) {
  os << config::toString(config);
  return os;
}
