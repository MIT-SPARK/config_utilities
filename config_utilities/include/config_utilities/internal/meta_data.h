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

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/checks.h"

namespace config::internal {

/**
 * @brief Struct that holds additional information about fields for printing.
 */
struct FieldInfo {
  // Name of the field. This is always given.
  std::string name;

  // Optional: Unit of the field.
  std::string unit;

  // The value of the field if the field is not a config.
  YAML::Node value;

  // Whether the field corresponds to its default value. Only queried if Settings().indicate_default_values is true.
  bool is_default = false;

  // Whether or not the field was parsed
  bool was_parsed = false;
};

// Struct to issue warnings. Currently used for parsing errors but can be extended to other warnings in the future.
struct Warning {
  virtual ~Warning() = default;
  Warning(const std::string& name, const std::string& message) : name_(name), message_(message) {}
  virtual std::string name() const { return name_; }
  virtual std::string message() const { return message_; }
  virtual std::unique_ptr<Warning> clone() const { return std::make_unique<Warning>(name_, message_); }

 private:
  std::string name_;
  std::string message_;
};

/**
 * @brief Meta-information struct that interfaces all the communication data when interacting with the configs. YAML is
 * used as internal data representation, so all conversions can be checked against yaml and all formatters, factories,
 * parsers can safely convert to yaml.
 */
struct MetaData {
  // Assignments and constructors.
  MetaData() = default;
  MetaData& operator=(const MetaData& other) {
    copyValues(other);
    return *this;
  }
  MetaData(const MetaData& other) { copyValues(other); }
  MetaData(MetaData&& other) = default;
  MetaData& operator=(MetaData&& other) = default;

  // Always get the name of the config if possible.
  std::string name;

  // Name of the field if the data is a sub-config.
  std::string field_name;

  // Whether the data stored belongs to a virtual config.
  bool is_virtual_config = false;

  // If this config is part of an array config, this is the index of the array.
  int array_config_index = -1;

  //! Map key if part of map config
  std::optional<std::string> map_config_key;

  // Yaml node used to get or set the data of a config.
  YAML::Node data;

  // All additional field information queried for printing.
  std::vector<FieldInfo> field_infos;

  // All checks that were performed on the config.
  std::vector<std::unique_ptr<CheckBase>> checks;

  // All error messages issued by the yaml parser (and potentially others in the future).
  std::vector<std::unique_ptr<Warning>> errors;

  // If a config has sub-configs, they are stored here.
  std::vector<MetaData> sub_configs;

  // Utility to look up if there's any error messages in the data or its sub-configs.
  bool hasErrors() const;

  // Utility to look up if any fields were not parsed
  bool hasMissing() const;

  // Utility function so not every class needs to write their own recursion.
  void performOnAll(const std::function<void(MetaData&)>& func);
  void performOnAll(const std::function<void(const MetaData&)>& func) const;

 private:
  void copyValues(const MetaData& other) {
    name = other.name;
    is_virtual_config = other.is_virtual_config;
    data = YAML::Clone(other.data);
    field_infos = other.field_infos;
    for (const auto& check : other.checks) {
      checks.emplace_back(check->clone());
    }
    for (const auto& error : other.errors) {
      errors.emplace_back(error->clone());
    }
    field_name = other.field_name;
    sub_configs = other.sub_configs;
    array_config_index = other.array_config_index;
    map_config_key = other.map_config_key;
  }
};

}  // namespace config::internal
