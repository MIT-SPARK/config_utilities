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
#include "config_utilities/internal/field_input_info.h"

namespace config::internal {

// Reserved token for virtual configs that are not set.
inline const std::string kUninitializedVirtualConfigType = "Uninitialized Virtual Config";

/**
 * @brief Struct that holds additional information about fields for printing.
 */
struct FieldInfo {
  //! Name of the field. This is always given.
  std::string name;

  //! The namespace when the field was parsed with respect to the visitor/meta_data containing this field.
  std::string ns;

  //! Optional: Unit of the field.
  std::string unit;

  //! The value of the field if the field is not a config.
  YAML::Node value;

  //! The default value of the field if the field is not a config.
  YAML::Node default_value;

  //! Whether the field corresponds to its default value. Only queried if Settings().printing.show_defaults is true.
  bool isDefault() const;

  //! Whether or not the field was parsed
  bool was_parsed = false;

  //! Whether or not the field is a meta field (e.g., type for virtual configs). If false, it is a proper field of the
  //! config.
  bool is_meta_field = false;

  //! Additional information about the input type and constraints of the field. Only queried when using getInfo.
  std::shared_ptr<FieldInputInfo> input_info;

  //! Serialize the field info to yaml.
  YAML::Node serializeFieldInfos() const;
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

  // Namespace of the config with respect to the root config
  std::string ns;

  // If the config is a virtual config, this is the type of the virtual config. If it is not set, the type will be the
  // uninitialized virtual config string.
  std::string virtual_config_type;

  // If the config is a virtual config, the own input info stores the available types.
  std::vector<std::string> available_types;

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

  // If true this is a config that is actually present. If false, this cnofig carries information about possible
  // configs, but these are not actually present.
  bool is_real_config = true;

  // Utility to look up if there's any error messages in the data or its sub-configs.
  bool hasErrors() const;

  // Utility to look up if any fields were not parsed
  bool hasMissing() const;

  // Utility function so not every class needs to write their own recursion.
  void performOnAll(const std::function<void(MetaData&)>& func);
  void performOnAll(const std::function<void(const MetaData&)>& func) const;

  // Check whether this is a virtual config.
  bool isVirtualConfig() const { return !virtual_config_type.empty(); }

  // Check whether this is a sub-confi in an array config.
  bool isArrayConfig() const { return array_config_index >= 0; }

  // Check whether this is a sub-config in a map config.
  bool isMapConfig() const { return map_config_key.has_value(); }

  // Utility function to get a display string for the index of a sub-config config.
  std::string displayIndex() const;

  // Find a matching subconfig in this meta data that matches the target subconfig from another meta data.
  MetaData* findMatchingSubConfig(const MetaData& search_key);
  const MetaData* findMatchingSubConfig(const MetaData& search_key) const;

  // Find a matching field info in this meta data that matches the target field info from another meta data.
  FieldInfo* findMatchingFieldInfo(const FieldInfo& search_key);
  const FieldInfo* findMatchingFieldInfo(const FieldInfo& search_key) const;

  // Utility function to get field info.
  YAML::Node serializeFieldInfos(bool include_meta_fields = false) const;

 private:
  void copyValues(const MetaData& other);
};

}  // namespace config::internal
