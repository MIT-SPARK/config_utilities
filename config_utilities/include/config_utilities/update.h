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

#include <string>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/types/enum.h"
#include "config_utilities/validation.h"

namespace config {

/**
 * @brief Update fields in a config.
 * @note This function will update the field and check the validity of the config afterwards. If the config is invalid,
 * the field will be reset to its original value.
 * @param node The node containing the fields to update.
 */
template <typename ConfigT>
bool updateFields(ConfigT& config, YAML::Node node, bool print_warnings = true, const std::string& name_space = "") {
  const auto backup = internal::Visitor::getValues(config);
  const internal::MetaData data =
      internal::Visitor::setValues(config, internal::lookupNamespace(node, name_space), print_warnings);
  const bool success = !data.hasErrors() && isValid(config);
  if (!success) {
    internal::Visitor::setValues(config, backup.data, print_warnings);
  }

  return success;
}

/**
 * @brief Update a field in a config.
 * @note This function will update the field and check the validity of the config afterwards. If the config is invalid,
 * the field will be reset to its original value.
 * @param field_name The name of the field to update.
 * @param value The new value of the field.
 */
template <typename T, typename ConfigT>
bool updateField(ConfigT& config,
                 const std::string& field_name,
                 const T& value,
                 bool print_warnings = true,
                 const std::string& name_space = "") {
  YAML::Node node;
  if constexpr (std::is_enum<T>::value) {
    internal::Logger::logWarning("Use updateFieldEnum for enum fields.");
    return false;
  } else {
    node[field_name] = value;
    return updateFields<ConfigT>(config, node, print_warnings, name_space);
  }
}

/**
 * @brief Update an enum field in a config.
 * @note This function will update the field and check the validity of the config afterwards. If the config is
 * invalid, the field will be reset to its original value.
 * @param field_name The name of the field to update.
 * @param value The new value of the field.
 */
template <typename ConfigT, typename EnumT>
bool updateFieldEnum(ConfigT& config,
                     const std::string& field_name,
                     const EnumT& value,
                     const std::map<EnumT, std::string>& enum_names = {},
                     bool print_warnings = true,
                     const std::string& name_space = "") {
  YAML::Node node;
  if constexpr (!std::is_enum<EnumT>::value) {
    internal::Logger::logWarning("Use updateField for non-enum fields.");
    return false;
  }

  std::string enum_name;
  if (enum_names.empty()) {
    enum_name = static_cast<int>(value);
  }

  if (enum_names.find(value) != enum_names.end()) {
    enum_name = enum_names.at(value);
  } else {
    internal::Logger::logWarning("Enum value not found in enum_names.");
    return false;
  }

  return updateField(config, field_name, enum_name, print_warnings, name_space);
}

/**
 * @brief Update an enum field in a config.
 * @note This function will update the field and check the validity of the config afterwards. If the config is
 * invalid, the field will be reset to its original value.
 * @param field_name The name of the field to update.
 * @param value The new value of the field.
 */
template <typename ConfigT, typename EnumT>
bool updateFieldEnum(ConfigT& config,
                     const std::string& field_name,
                     const EnumT& value,
                     std::vector<std::string> enum_names,
                     bool print_warnings = true,
                     const std::string& name_space = "") {
  YAML::Node node;
  if constexpr (!std::is_enum<EnumT>::value) {
    internal::Logger::logWarning("Use updateField for non-enum fields.");
    return false;
  }

  const auto enum_map = createEnumMap<EnumT>(enum_names);
  return updateFieldEnum<ConfigT, EnumT>(config, field_name, value, enum_map, print_warnings, name_space);
}

}  // namespace config
