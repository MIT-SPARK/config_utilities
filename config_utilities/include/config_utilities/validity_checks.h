#pragma once

#include <string>
#include <utility>
#include <vector>
#include <sstream>
#include<stdexcept>

#include <glog/logging.h>

#include "config_utilities/internal/meta_data.h"
#include "config_utilities/internal/validity_checker.h"
#include "config_utilities/traits.h"

namespace config {

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
      LOG(WARNING) << "Can not use 'config::isValid()' on non-config T='" << typeid(ConfigT).name()
                   << "'. Please implement 'void declare_config(T&)' for your struct.";
    }
    return false;
  }
  internal::MetaData data = internal::MetaData::create();
  data.mode = internal::MetaData::Mode::kCheckValid;
  data.validity_checker.reset();

  // Run the checks call as defined in the config declaration function.
  // NOTE: We know that in mode kCheckValid, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));

  // Extract the result and print the warnings if requested.
  data.validity_checker.setName(data.name);
  return data.validity_checker.isValid(print_warnings);
};

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
    throw std::runtime_error(ss.str());
  }
  internal::MetaData data = internal::MetaData::create();
  data.mode = internal::MetaData::Mode::kCheckValid;
  data.validity_checker.reset();

  // Run the checks call as defined in the config declaration function.
  // NOTE: We know that in mode kCheckValid, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));

  // Extract the result and print the warnings if requested.
  data.validity_checker.setName(data.name);
  data.validity_checker.checkValid();
};

/**
 * @brief Assert that a config is valid. This will terminate the program if invalid. Return the config if it is valid.
 *
 * @tparam ConfigT The config type.
 * @param config The config to check.
 * @returns The validated config.
 */
template <typename ConfigT>
const ConfigT& checkValid(const ConfigT& config) {
  if (!isConfig<ConfigT>()) {
    std::stringstream ss;
     ss << "Can not use 'config::checkValid()' on non-config T='" << typeid(ConfigT).name()
                 << "'. Please implement 'void declare_config(T&)' for your struct.";
    throw std::runtime_error(ss.str());
  }
  internal::MetaData data = internal::MetaData::create();
  data.mode = internal::MetaData::Mode::kCheckValid;
  data.validity_checker.reset();

  // Run the checks call as defined in the config declaration function.
  // NOTE: We know that in mode kCheckValid, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));

  // Extract the result and print the warnings if requested.
  data.validity_checker.setName(data.name);
  data.validity_checker.checkValid();
  return config;
};

}  // namespace config