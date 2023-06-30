#pragma once

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/meta_data.h"

namespace config {

namespace internal {

// Function that produces the a vector of strings for each field of a config.
template <typename ConfigT>
std::string toStringInternal(const ConfigT& config, int indent) {
  const int indent_prev = meta_data_->indent;
  meta_data_->indent = indent;

  meta_data_->messages = std::make_unique<std::vector<std::string>>();

  // Create the default values if required.
  if (Settings::instance().indicate_default_values) {
    ConfigT defaults;
    MetaData& data = MetaData::instance();
    data.get_defaults = true;
    
    meta_data_->default_values = std::make_unique<std::unordered_map<std::string, std::string>>(defaults->getValues());
    meta_data_->use_printing_to_get_values = true;
    meta_data_->merged_setup_already_used = true;

    // NOTE: setupParamsAndPrinting() does not modify 'this' in printing mode.
    ((ConfigInternal*)this)->setupParamsAndPrinting();
    printFields();
  }

  meta_data_->use_printing_to_get_values = false;
  meta_data_->merged_setup_already_used = true;

  // NOTE: setupParamsAndPrinting() does not modify 'this' in printing mode.
  ((ConfigInternal*)this)->setupParamsAndPrinting();
  printFields();
  std::string result;
  for (const std::string& msg : *(meta_data_->messages)) {
    result.append("\n" + msg);
  }
  if (!result.empty()) {
    result = result.substr(1);
  }
  meta_data_->messages.reset(nullptr);
  meta_data_->default_values.reset(nullptr);
  meta_data_->indent = indent_prev;
  meta_data_->merged_setup_currently_active = false;
  meta_data_->global_printing_processed = true;
  return result;
};

}  // namespace internal

template <typename ConfigT>
std::string toString(const ConfigT& config) {
  if (!isConfig<ConfigT>()) {
    LOG(WARNING) << "Can not use 'config::toString()' on non-config T='" << typeid(ConfigT).name()
                 << "'. Please implement 'void declare_config(T&)' for your struct.";
    return "";
  }
  internal::MetaData data = internal::MetaData::create();
  data.mode = internal::MetaData::Mode::kToString;
  data.messages.clear();

  // Run the checks call as defined in the config declaration function.
  // NOTE: We know that in mode kCheckValid, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));
};

/**
 * @brief Produces a printable summary of the config as string, containing its
 * name and all parameter values.
 */
std::string toString() {
  std::string result = internal::printCenter(name_, GlobalSettings::instance().print_width, '=') + "\n" +
                       toStringInternal(meta_data_->indent) + "\n" +
                       std::string(GlobalSettings::instance().print_width, '=');
  meta_data_->messages.reset(nullptr);
  return result;
};

}  // namespace config