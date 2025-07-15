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

#include "config_utilities/internal/meta_data.h"

#include "config_utilities/internal/string_utils.h"
#include "config_utilities/settings.h"

namespace config::internal {

bool FieldInfo::isDefault() const {
  // NOTE(lschmid): Operator YAML::Node== checks for identity, not equality. Since these are all scalars, comparing
  // the formatted strings should be identical.
  return internal::yamlToString(value) == internal::yamlToString(default_value);
}

bool MetaData::hasErrors() const {
  if (!errors.empty()) {
    return true;
  }
  for (const auto& sub_config : sub_configs) {
    if (sub_config.hasErrors()) {
      return true;
    }
  }
  return false;
}

bool MetaData::hasMissing() const {
  // first check all current fields
  for (const auto& field_info : field_infos) {
    if (!field_info.was_parsed) {
      return true;
    }
  }

  // next check all subconfigs recursively
  for (const auto& sub_config : sub_configs) {
    if (sub_config.hasMissing()) {
      return true;
    }
  }

  return false;
}

void MetaData::performOnAll(const std::function<void(MetaData&)>& func) {
  func(*this);
  for (MetaData& sub_config : sub_configs) {
    sub_config.performOnAll(func);
  }
}

void MetaData::performOnAll(const std::function<void(const MetaData&)>& func) const {
  func(*this);
  for (const MetaData& sub_config : sub_configs) {
    sub_config.performOnAll(func);
  }
}

YAML::Node FieldInfo::serializeFieldInfos() const {
  YAML::Node result;
  result["type"] = "field";
  result["name"] = name;
  if (!unit.empty()) {
    result["unit"] = unit;
  }
  result["value"] = YAML::Clone(value);
  result["default"] = YAML::Clone(default_value);
  if (was_parsed) {
    result["was_parsed"] = true;
  }
  if (input_info) {
    result["input_info"] = input_info->toYaml();
  }
  return result;
}

YAML::Node MetaData::serializeFieldInfos() const {
  YAML::Node result;
  // Log the config.
  result["type"] = "config";
  result["name"] = name;
  if (!field_name.empty()) {
    result["field_name"] = field_name;
  }
  if (is_virtual_config) {
    result["available_types"] = available_types;
    if (data[field_name]) {
      result["name"] = data[field_name][Settings::instance().factory.type_param_name];
    } else {
      result["name"] = "Uninitialized Virtual Config";  // Reserved token for virtual configs that are not set.
    }
  }
  if (array_config_index >= 0) {
    result["array_index"] = array_config_index;
  }
  if (map_config_key) {
    result["map_config_key"] = map_config_key.value();
  }
  YAML::Node fields(YAML::NodeType::Sequence);

  // Parse the direct fields.
  for (const FieldInfo& info : field_infos) {
    fields.push_back(info.serializeFieldInfos());
  }

  // Parse the sub-configs.
  for (const MetaData& sub_data : sub_configs) {
    fields.push_back(sub_data.serializeFieldInfos());
  }

  result["fields"] = fields;
  return result;
}

}  // namespace config::internal
