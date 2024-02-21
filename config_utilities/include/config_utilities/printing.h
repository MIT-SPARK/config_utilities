/** -----------------------------------------------------------------------------
 * Copyright (c) 2023 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
 * AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2023
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

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
 * @param config The config to print.
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
 * @param config The config to print.
 * @param print_warnings If true, prints warnings for any failed conversions.
 * @returns The string representation of the config.
 */
template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type = true>
std::string toString(const std::vector<ConfigT>& array_config, bool print_warnings = true) {
  // Get the data of the config.
  internal::MetaData data = internal::Visitor::getValues(array_config);
  // force names for config types
  data.name = "Config Array";
  for (auto& sub_config : data.sub_configs) {
    sub_config.field_name = "config_array";
  }

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
 * @tparam K The key type for the map
 * @tparam ConfigT The type of the config to print.
 * @param config The config to print.
 * @param print_warnings If true, prints warnings for any failed conversions.
 * @returns The string representation of the config.
 */
template <typename K, typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type = true>
std::string toString(const std::map<K, ConfigT>& config, bool print_warnings = true) {
  // Get the data of the config.
  internal::MetaData data = internal::Visitor::getValues(config);
  // force names for config types
  data.name = "Config Map";
  for (auto& sub_config : data.sub_configs) {
    sub_config.field_name = "config_map";
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
