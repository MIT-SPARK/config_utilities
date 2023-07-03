#pragma once

#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config {

/**
 * @brief Returns a string representation of the config.
 *
 * @tparam ConfigT The type of the config to print.
 * @param config The config to print.
 * @param logger [Advanced use] Optionally pass a specific logger to log this call to.
 * @param formatter [Advanced use] Optionally pass a specific formatter to format the warnings with.
 * @returns The string representation of the config.
 */
template <typename ConfigT>
std::string toString(const ConfigT& config,
                     internal::Logger::Ptr logger = internal::Logger::defaultLogger(),
                     internal::Formatter::Ptr formatter = internal::Formatter::defaultFormatter()) {
  if (!isConfig<ConfigT>()) {
    std::stringstream ss;
    ss << "Can not use 'config::toString()' on non-config T='" << typeid(ConfigT).name()
       << "'. Please implement 'void declare_config(T&)' for your struct.";
    logger->logError(ss.str());
    return "";
  }
  // Get the data of the config.
  internal::MetaData data = internal::Visitor::getValues(config);

  // If requested get all default values.
  if (Settings().indicate_default_values) {
    // ConfigT defaults;
    // const internal::MetaData default_data = internal::Visitor::getValues(defaults);
    // for (const auto& kv : data.data) {
    //   std::cout << kv.first.as<std::string>() << std::endl;  // prints Foo
    //   const YAML::Node& value = kv.second;              // the value
    // if (default_data.data.find(key) != default_data.data.end()) {
    //   data.params_using_defaults.emplace_back(key);
    // }
  }
  return formatter->formatToString(data);
}

}  // namespace config
