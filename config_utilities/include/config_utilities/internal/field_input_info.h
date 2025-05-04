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

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace config::internal {

/**
 * @brief Information about the input type and constraints of a field.
 */
struct FieldInputInfo {
  using Ptr = std::shared_ptr<FieldInputInfo>;
  //! Type of the field input. Anything that is not specialized can be parsed as kYAML.
  enum class Type { kBool, kInt, kFloat, kString, kOptions, kYAML } type;
  static std::string typeToString(Type type);

  explicit FieldInputInfo(Type type) : type(type) {}
  virtual ~FieldInputInfo() = default;

  //! Convert the input info to yaml format for serialization.
  virtual YAML::Node toYaml() const;

  //! Merge the input info with another one. This is used to combine constraints from different sources.
  static Ptr merge(const FieldInputInfo::Ptr& from, const FieldInputInfo::Ptr& to);

  // Utility interface to set int/float constraints.
  virtual void setMin(YAML::Node /* min */, bool /* lower_inclusive */ = true) {}
  virtual void setMax(YAML::Node /* max */, bool /* upper_inclusive */ = true) {}

 private:
  // Implementation of merging for the same type.
  virtual void mergeSame(const FieldInputInfo& /* other */) {}
};

struct IntFieldInputInfo : public FieldInputInfo {
  IntFieldInputInfo(const std::string& type_str) : FieldInputInfo(Type::kInt), type_str(type_str) {}

  // Constraints for the field.
  // NOTE(lschmid): We currently do not consider data larger than 64 bit integers.
  std::string type_str;  // "int8", "int16", "int32", "int64", "uint8", "uint16", "uint32", "uint64"
  std::optional<int64_t> min;
  std::optional<uint64_t> max;
  bool lower_inclusive = true;
  bool upper_inclusive = true;

  YAML::Node toYaml() const override;
  void mergeSame(const FieldInputInfo& other) override;
  void setMin(YAML::Node min, bool lower_inclusive = true) override;
  void setMax(YAML::Node max, bool upper_inclusive = true) override;
};

struct FloatFieldInputInfo : public FieldInputInfo {
  FloatFieldInputInfo(const std::string& type_str) : FieldInputInfo(Type::kFloat), type_str(type_str) {}

  // Constraints for the field.
  // NOTE(lschmid): We currently do not consider data larger than 64 bit floats.
  std::string type_str;  // "float32", "float64"
  std::optional<double> min;
  std::optional<double> max;
  bool lower_inclusive = true;
  bool upper_inclusive = true;

  YAML::Node toYaml() const override;
  void mergeSame(const FieldInputInfo& other) override;
  void setMin(YAML::Node min, bool lower_inclusive = true) override;
  void setMax(YAML::Node max, bool upper_inclusive = true) override;
};

struct OptionsFieldInputInfo : public FieldInputInfo {
  OptionsFieldInputInfo() : FieldInputInfo(Type::kOptions) {}

  // The possible options for the field. Note that since these will anyways be stringified, all types can be stored as
  // strings.
  std::vector<std::string> options;

  YAML::Node toYaml() const override;
  void mergeSame(const FieldInputInfo& other) override;
};

/**
 * @brief Create field info based on common types.
 */
template <typename T>
FieldInputInfo::Ptr createFieldInputInfo() {
  // Default anything not specialized to YAML.
  return std::make_shared<FieldInputInfo>(FieldInputInfo::Type::kYAML);
}

// Specializations for common types.
// Bool.
template <>
FieldInputInfo::Ptr createFieldInputInfo<bool>();

// Ints.
template <>
FieldInputInfo::Ptr createFieldInputInfo<int8_t>();

template <>
FieldInputInfo::Ptr createFieldInputInfo<int16_t>();

template <>
FieldInputInfo::Ptr createFieldInputInfo<int32_t>();

template <>
FieldInputInfo::Ptr createFieldInputInfo<int64_t>();

template <>
FieldInputInfo::Ptr createFieldInputInfo<uint8_t>();

template <>
FieldInputInfo::Ptr createFieldInputInfo<uint16_t>();

template <>
FieldInputInfo::Ptr createFieldInputInfo<uint32_t>();

template <>
FieldInputInfo::Ptr createFieldInputInfo<uint64_t>();

// Floats.
template <>
FieldInputInfo::Ptr createFieldInputInfo<float>();

template <>
FieldInputInfo::Ptr createFieldInputInfo<double>();

// Strings.
template <>
FieldInputInfo::Ptr createFieldInputInfo<std::string>();

}  // namespace config::internal
