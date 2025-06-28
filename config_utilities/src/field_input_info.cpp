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
#include "config_utilities/internal/field_input_info.h"

#include <algorithm>
#include <iostream>  // TMP
#include <unordered_set>

#include "config_utilities/internal/yaml_parser.h"

namespace config::internal {

std::string FieldInputInfo::typeToString(Type type) {
  switch (type) {
    case Type::kBool:
      return "bool";
    case Type::kInt:
      return "int";
    case Type::kFloat:
      return "float";
    case Type::kString:
      return "string";
    case Type::kOptions:
      return "options";
    case Type::kYAML:
      return "yaml";
  }
  return "unknown";
}

YAML::Node FieldInputInfo::toYaml() const {
  YAML::Node node;
  node["type"] = typeToString(type);
  return node;
}

FieldInputInfo::Ptr FieldInputInfo::merge(const FieldInputInfo::Ptr& from, const FieldInputInfo::Ptr& to) {
  if (!from) {
    return to;
  }
  if (!to) {
    return from;
  }

  if (from->type == to->type) {
    to->mergeSame(*from);
    return to;
  }

  if (to->type == FieldInputInfo::Type::kOptions) {
    // Options will always overwrite other types.
    return to;
  }
  if (from->type == FieldInputInfo::Type::kOptions) {
    return from;
  }

  // For general conflicts resort to YAML and let the configs sort it out.
  return std::make_shared<FieldInputInfo>(FieldInputInfo::Type::kYAML);
}

YAML::Node IntFieldInputInfo::toYaml() const {
  YAML::Node node;
  node["type"] = "int";
  node["min"] = min;
  node["max"] = max;
  // Only store the rarer cases.
  if (!lower_inclusive) {
    node["lower_exclusive"] = true;
  }
  if (!upper_inclusive) {
    node["upper_exclusive"] = true;
  }
  return node;
}

void IntFieldInputInfo::mergeSame(const FieldInputInfo& other) {
  const auto& other_info = dynamic_cast<const IntFieldInputInfo&>(other);
  if (min > other_info.min) {
    min = other_info.min;
    lower_inclusive = other_info.lower_inclusive;
  } else if (min == other_info.min) {
    lower_inclusive = lower_inclusive && other_info.lower_inclusive;
  }
  if (max < other_info.max) {
    max = other_info.max;
    upper_inclusive = other_info.upper_inclusive;
  } else if (max == other_info.max) {
    upper_inclusive = upper_inclusive && other_info.upper_inclusive;
  }
}

void IntFieldInputInfo::setMin(YAML::Node min, bool lower_inclusive) {
  auto val = YamlParser::fromYaml<int64_t>(min);
  if (!val) {
    std::cout << "Failed to parse min int value: " << min << std::endl;
    return;
  }
  this->min = *val;
  this->lower_inclusive = lower_inclusive;
  std::cout << "Set int min: " << this->min << std::endl;
}

void IntFieldInputInfo::setMax(YAML::Node max, bool upper_inclusive) {
  auto val = YamlParser::fromYaml<uint64_t>(max);
  if (!val) {
    std::cout << "Failed to parse max int value: " << min << std::endl;
    return;
  }
  this->max = *val;
  this->upper_inclusive = upper_inclusive;
  std::cout << "Set int max: " << this->max << std::endl;
}

YAML::Node FloatFieldInputInfo::toYaml() const {
  YAML::Node node;
  node["type"] = "float";
  node["min"] = min;
  node["max"] = max;
  if (!lower_inclusive) {
    node["lower_exclusive"] = true;
  }
  if (!upper_inclusive) {
    node["upper_exclusive"] = true;
  }
  return node;
}

void FloatFieldInputInfo::mergeSame(const FieldInputInfo& other) {
  const auto& other_info = dynamic_cast<const FloatFieldInputInfo&>(other);
  if (min > other_info.min) {
    min = other_info.min;
    lower_inclusive = other_info.lower_inclusive;
  } else if (min == other_info.min) {
    lower_inclusive = lower_inclusive && other_info.lower_inclusive;
  }
  if (max < other_info.max) {
    max = other_info.max;
    upper_inclusive = other_info.upper_inclusive;
  } else if (max == other_info.max) {
    upper_inclusive = upper_inclusive && other_info.upper_inclusive;
  }
}

void FloatFieldInputInfo::setMin(YAML::Node min, bool lower_inclusive) {
  auto val = YamlParser::fromYaml<double>(min);
  if (!val) {
    std::cout << "Failed to parse min float value: " << min << std::endl;
    return;
  }
  this->min = *val;
  this->lower_inclusive = lower_inclusive;
  std::cout << "Set float min: " << this->min << std::endl;
}

void FloatFieldInputInfo::setMax(YAML::Node max, bool upper_inclusive) {
  auto val = YamlParser::fromYaml<double>(max);
  if (!val) {
    std::cout << "Failed to parse max float value: " << min << std::endl;
    return;
  }
  this->max = *val;
  this->upper_inclusive = upper_inclusive;
  std::cout << "Set float max: " << this->max << std::endl;
}

YAML::Node OptionsFieldInputInfo::toYaml() const {
  YAML::Node node;
  node["type"] = "options";
  node["options"] = options;
  return node;
}

void OptionsFieldInputInfo::mergeSame(const FieldInputInfo& other) {
  const auto& other_info = dynamic_cast<const OptionsFieldInputInfo&>(other);
  // Intersection of options.
  std::unordered_set<std::string> prev_options(options.begin(), options.end());
  options.clear();
  for (const auto& option : other_info.options) {
    if (prev_options.count(option)) {
      options.push_back(option);
    }
  }
}

// Helper function for int types.
template <typename InfoT, typename NumericT>
FieldInputInfo::Ptr createNumericInfo() {
  auto info = std::make_shared<InfoT>();
  info->min = std::numeric_limits<NumericT>::lowest();
  info->max = std::numeric_limits<NumericT>::max();
  return info;
}

// Bool.
template <>
FieldInputInfo::Ptr createFieldInputInfo<bool>() {
  return std::make_shared<FieldInputInfo>(FieldInputInfo::Type::kBool);
}

// Ints.
template <>
FieldInputInfo::Ptr createFieldInputInfo<int8_t>() {
  return createNumericInfo<IntFieldInputInfo, int16_t>();
}

template <>
FieldInputInfo::Ptr createFieldInputInfo<int16_t>() {
  return createNumericInfo<IntFieldInputInfo, int16_t>();
}

template <>
FieldInputInfo::Ptr createFieldInputInfo<int32_t>() {
  return createNumericInfo<IntFieldInputInfo, int32_t>();
}

template <>
FieldInputInfo::Ptr createFieldInputInfo<int64_t>() {
  return createNumericInfo<IntFieldInputInfo, int64_t>();
}

template <>
FieldInputInfo::Ptr createFieldInputInfo<uint8_t>() {
  return createNumericInfo<IntFieldInputInfo, uint8_t>();
}

template <>
FieldInputInfo::Ptr createFieldInputInfo<uint16_t>() {
  return createNumericInfo<IntFieldInputInfo, uint16_t>();
}

template <>
FieldInputInfo::Ptr createFieldInputInfo<uint32_t>() {
  return createNumericInfo<IntFieldInputInfo, uint32_t>();
}

template <>
FieldInputInfo::Ptr createFieldInputInfo<uint64_t>() {
  return createNumericInfo<IntFieldInputInfo, uint64_t>();
}

// Floats.
template <>
FieldInputInfo::Ptr createFieldInputInfo<float>() {
  return std::make_shared<FloatFieldInputInfo>();
}

template <>
FieldInputInfo::Ptr createFieldInputInfo<double>() {
  return std::make_shared<FloatFieldInputInfo>();
}

// Strings.
template <>
FieldInputInfo::Ptr createFieldInputInfo<std::string>() {
  return std::make_shared<FieldInputInfo>(FieldInputInfo::Type::kString);
}

}  // namespace config::internal
