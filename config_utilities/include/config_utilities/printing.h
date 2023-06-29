#pragma once

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/config.h"

namespace config {


template <typename ConfigT>
std::string toString(ConfigT& config) {
  if (!isConfig<ConfigT>()) {
    LOG(WARNING) << "Can not use 'config::toString(T&)' on non-config T='" << typeid(ConfigT).name()
                 << "'. Please implement 'void declare_config(T&)' for your struct.";
    return "";
  }
  internal::MetaData data = internal::MetaData::create();
  data.mode = internal::MetaData::Mode::kToString;
  declare_config(config);
  return "Name: " + data.name;
};

}  // namespace config