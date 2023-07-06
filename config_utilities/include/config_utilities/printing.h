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
 * @tparam ConfigT The type of the config to print.
 * @param config The config to print.
 * @param print_warnings If true, prints warnings for any failed conversions.
 * @returns The string representation of the config.
 */
template <typename ConfigT>
std::string toString(const ConfigT& config, bool print_warnings = true) {
  if (!isConfig<ConfigT>()) {
    std::stringstream ss;
    ss << "Can not use 'config::toString()' on non-config T='" << typeid(ConfigT).name()
       << "'. Please implement 'void declare_config(T&)' for your struct.";
    internal::Logger::logError(ss.str());
    return "";
  }
  // Get the data of the config.
  internal::MetaData data = internal::Visitor::getValues(config);

  // If requested check default values by comparing against a default constructed config.
  if (Settings().indicate_default_values) {
    ConfigT defaults;
    // NOTE(lschmid): Operator YAML::Node== checks for identity, not equality. Comparing the formatted strings should be
    // identical for default constructed configs. Check for all subconfigs using the fact that the structure is
    // identical.
    const internal::MetaData default_data = internal::Visitor::getValues(defaults, false);
    std::vector<const internal::MetaData*> default_configs;
    default_data.performOnAll([&default_configs](const internal::MetaData& data) { default_configs.push_back(&data); });
    std::vector<internal::MetaData*> configs;
    data.performOnAll([&configs](internal::MetaData& data) { configs.push_back(&data); });

    for (size_t i = 0; i < configs.size(); ++i) {
      for (internal::FieldInfo& info : configs[i]->field_infos) {
        if (internal::dataToString(configs[i]->data[info.name]) ==
            internal::dataToString(default_configs[i]->data[info.name])) {
          info.is_default = true;
        }
      }
    }
  }

  // Format the output data.
  if (print_warnings && data.hasErrors()) {
    internal::Logger::logWarning(
        internal::Formatter::formatErrors(data, "Errors parsing config", internal::Formatter::Severity::kWarning));
  }

  return internal::Formatter::formatToString(data);
}

}  // namespace config

// Define the ostream operator for declared configs.
template <typename ConfigT,
          typename std::enable_if<config::internal::is_config_impl<ConfigT>::value, bool>::type = true>
std::ostream& operator<<(std::ostream& os, const ConfigT& config) {
  os << config::toString(config);
  return os;
}
