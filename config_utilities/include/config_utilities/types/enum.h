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

#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"

namespace config {

/**
 * @brief Create a map of enum values to their string names. This is useful for defining the enum names in a single
 * location, and then using them for the enum definition and parsing.
 * @tparam EnumT The enum type.
 * @param enum_names List of all possible enum names in identical order to the enum definition. Use only with sequential
 * (=default valued uint type) enums.
 * @return Map of enum values to their string names.
 */
template <typename EnumT>
std::map<EnumT, std::string> createEnumMap(const std::vector<std::string>& enum_names) {
  std::map<EnumT, std::string> enum_map;
  for (size_t i = 0; i < enum_names.size(); ++i) {
    enum_map[static_cast<EnumT>(i)] = enum_names[i];
  }
  return enum_map;
}


/**
 * @brief A struct that provides conversion between an ennum type and its string representation. The enum definition can
 * provides interfaces for user-side conversion, and is an automatic type converter for config field parsing.
 * @tparam EnumT The enum type to be registered for conversion.
 */
template <typename EnumT>
struct Enum {
  // Interfaces to for user-side conversion.

  /**
   * @brief Convert an enum value to its string representation.
   * @param value The enum value to be converted.
   * @param print_errors If true, an error message is printed if the value is not a set enum value.
   * @return The string representation of the enum value if the value is in the defined values. Returns '<Invalid Enum
   * Value>' otherwise. This should not happen if all enum values are defined using 'setNames()'.
   */
  static std::string toString(EnumT value, bool print_errors = true) {
    std::string result;
    std::lock_guard<std::mutex> lock(instance().mutex_);
    if (!instance().parse(value, result) && print_errors) {
      internal::Logger::logWarning("Enum conversion failed: Value '" + std::to_string(static_cast<int>(value)) +
                                   "' is out of bounds for EnumT='" + internal::typeName<EnumT>() + "' with values [" +
                                   instance().printValueList() + "].");
    }
    return result;
  }

  /**
   * @brief Convert a enum string representation to its enum value.
   * @param value The string representation of the enum.
   * @param print_errors If true, an error message is printed if the string is not a set enum name.
   * @returns The enum value if the string is in the defined names. Returns the default value of the enum otherwise.
   */
  static EnumT fromString(const std::string& value, bool print_errors = true) {
    auto result = EnumT();
    std::lock_guard<std::mutex> lock(instance().mutex_);
    if (!instance().parse(value, result) && print_errors) {
      internal::Logger::logWarning("Enum conversion failed: Name '" + value + "' is out of bounds for EnumT='" +
                                   internal::typeName<EnumT>() + "' with names [" + instance().printNameList() + "].");
    }
    return result;
  }

  /**
   * @brief Define the names of the enum values. This is used for conversion between enum values and strings. This
   * function overwrites all previous enum name definitions. The complete list of enum names should be defined before
   * any enums are parsed.
   * @param enum_names Map of enum values to their string names.
   */
  static void setNames(const std::map<EnumT, std::string>& enum_names) {
    std::lock_guard<std::mutex> lock(instance().mutex_);
    instance().enum_names_ = enum_names;
  }

  /**
   * @brief Define the names of the enum values via static initialization struct. This is used for conversion between
   * enum values and strings. This function overwrites all previous enum name definitions. The complete list of enum
   * names should be defined before any enums are parsed.
   */
  struct Initializer {
    explicit Initializer(const std::map<EnumT, std::string>& enum_names) { setNames(enum_names); }
  };

 private:
  friend internal::Visitor;
  template <typename T>
  friend void enum_field(T&, const std::string&, const std::map<T, std::string>&);

  // Singleton implementation as initialization order of static variables is not guaranteed.
  Enum() = default;

  static Enum<EnumT>& instance() {
    static Enum<EnumT> instance;
    return instance;
  }

  // Interfaces to work as a type converter for config field parsing.
  static std::string toIntermediate(EnumT value, std::string& error) {
    std::string result;
    std::lock_guard<std::mutex> lock(instance().mutex_);
    if (!instance().parse(value, result)) {
      error = "Value '" + std::to_string(static_cast<int>(value)) + "' is out of bounds for enum with values [" +
              instance().printValueList() + "]";
    }
    return result;
  }

  static void fromIntermediate(const std::string& intermediate, EnumT& value, std::string& error) {
    std::lock_guard<std::mutex> lock(instance().mutex_);
    if (!instance().parse(intermediate, value)) {
      error = "Name '" + intermediate + "' is out of bounds for enum with names [" + instance().printNameList() + "]";
    }
  }

  // Tools to print the enum names and values for error messages. These are therefore not locked.
  static std::string printNameList() {
    std::string result;
    for (const auto& [_, name] : instance().enum_names_) {
      result += "'" + name + "', ";
    }
    if (!result.empty()) {
      result.erase(result.size() - 2);
    }
    return result;
  }

  static std::string printValueList() {
    std::string result;
    for (const auto& [value, _] : instance().enum_names_) {
      result += "'" + std::to_string(static_cast<int>(value)) + "', ";
    }
    if (!result.empty()) {
      result.erase(result.size() - 2);
    }
    return result;
  }

  // Parsing implementation.
  static bool parse(const EnumT value, std::string& result) {
    const auto it = instance().enum_names_.find(value);
    if (it != instance().enum_names_.end()) {
      result = it->second;
      return true;
    }
    result = "<Invalid Enum Value>";
    return false;
  }

  static bool parse(const std::string& name, EnumT& result) {
    const auto it = std::find_if(instance().enum_names_.begin(),
                                 instance().enum_names_.end(),
                                 [&name](const auto& pair) { return pair.second == name; });
    if (it != instance().enum_names_.end()) {
      result = it->first;
      return true;
    }
    return false;
  }

  // Global data for the enum.
  std::map<EnumT, std::string> enum_names_;
  // NOTE(lschmid): This is currently implemented with a global mutex, such that the declared enums can be used across
  // threads. Since config code is typically not called frequently, this should not be a performance issue.
  std::mutex mutex_;
};

// Simplified interfaces to include in 'declare_config'.

/**
 * @brief Declare a field of a config to be an enum, that will be parsed and checked bystring  value names. Note that
 * this allows specifying other values than defined in the global `Enum<EnumT>` converter definition, and will not
 * modify the global definition.
 *
 * @tparam EnumT The enum type.
 * @param field The config member that stores data.
 * @param field_name The name of the field.
 * @param enum_names Map of enum values to names for non-sequential enums. If empty, the global enum definition is used.
 */
template <typename EnumT>
void enum_field(EnumT& field, const std::string& field_name, const std::map<EnumT, std::string>& enum_names = {}) {
  if (enum_names.empty()) {
    internal::Visitor::visitField<Enum<EnumT>>(field, field_name, "");
    return;
  }
  auto& converter = Enum<EnumT>::instance();
  converter.mutex_.lock();
  const auto backup = std::move(converter.enum_names_);
  converter.enum_names_ = enum_names;
  converter.mutex_.unlock();
  internal::Visitor::visitField<Enum<EnumT>>(field, field_name, "");
  converter.mutex_.lock();
  converter.enum_names_ = std::move(backup);
  converter.mutex_.unlock();
}

/**
 * @brief Declare a field of a config to be an enum, that will be parsed and checked bystring  value names. Note that
 * this allows specifying other values than defined in the global `Enum<EnumT>` converter definition, and will not
 * modify the global definition.
 *
 * @tparam EnumT The enum type.
 * @param field The config member that stores data.
 * @param field_name The name of the field.
 * @param enum_names List of all possible enum names in identical order to the enum definition. Use only with sequential
 * (=default valued uint type) enums.
 */
template <typename EnumT>
void enum_field(EnumT& field, const std::string& field_name, const std::vector<std::string>& enum_names) {
  const auto enum_map = createEnumMap<EnumT>(enum_names);
  enum_field(field, field_name, enum_map);
}

}  // namespace config
