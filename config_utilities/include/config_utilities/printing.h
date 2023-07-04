#pragma once

#include <sstream>
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
 * @returns The string representation of the config.
 */
template <typename ConfigT>
std::string toString(const ConfigT& config) {
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
    // NOTE(lschmid): Operator YAML::Node== checks for identity,not equality. Comparing the formatted strings should be
    // identical for default constructed configs.
    const internal::MetaData default_data = internal::Visitor::getValues(defaults);
    for (internal::FieldInfo& info : data.field_info) {
      if (internal::dataToString(data.data[info.name], info.type_info) ==
          internal::dataToString(default_data.data[info.name], info.type_info)) {
        info.is_default = true;
      }
    }
  }

  // Format the output data.
  return internal::Formatter::formatToString(data);
}

}  // namespace config
