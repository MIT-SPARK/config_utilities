#pragma once

#include <map>
#include <mutex>
#include <string>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"

namespace config {

namespace internal {
struct Visitor;
}  // namespace internal

/**
 * @brief A struct that provides conversion between an ennum type and its string representation. The enum definition can
 * provides interfaces for user-side conversion, and is an automatic type converter for config field parsing.
 * @tparam EnumT The enum type to be registered for conversion.
 */
template <typename EnumT>
struct Enum {
  // Set the enum names via creation of a static instance of this struct.
  Enum() = default;
  explicit Enum(const std::map<EnumT, std::string>& enum_names) { setNames(enum_names); }

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
    std::lock_guard<std::mutex> lock(mutex_);
    if (!parse(value, result) && print_errors) {
      internal::Logger::logWarning("Enum conversion failed: Value '" + std::to_string(static_cast<int>(value)) +
                                   "' is out of bounds for EnumT='" + internal::typeName<EnumT>() + "' with values [" +
                                   enum_values_list_ + "].");
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
    EnumT result;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!parse(value, result) && print_errors) {
      internal::Logger::logWarning("Enum conversion failed: Name '" + value + "' is out of bounds for EnumT='" +
                                   internal::typeName<EnumT>() + "' with names [" + enum_names_list_ + "].");
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
    std::lock_guard<std::mutex> lock(mutex_);
    enum_names_ = enum_names;
    enum_names_list_ = "";
    enum_values_list_ = "";
    for (const auto& [value, name] : enum_names_) {
      enum_names_list_ += "'" + name + "', ";
      enum_values_list_ + "'" + std::to_string(static_cast<int>(value)) + "', ";
    }
    enum_names_list_.erase(enum_names_list_.size() - 2);
    enum_values_list_.erase(enum_values_list_.size() - 2);
  }

 private:
  friend internal::Visitor;

  // Interfaces to work as a type converter for config field parsing.
  static std::string toIntermediate(EnumT value, std::string& error) {
    std::string result;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!parse(value, result)) {
      error = "Value '" + std::to_string(static_cast<int>(value)) + "' is out of bounds for enum with values [" +
              enum_values_list_ + "]";
    }
    return result;
  }

  static EnumT fromIntermediate(const std::string& value, std::string& error) {
    EnumT result;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!parse(value, result)) {
      error = "Name '" + value + "' is out of bounds for enum with names [" + enum_names_list_ + "]";
    }
    return result;
  }

  // Parsing implementation.
  static bool parse(const EnumT value, std::string& result) {
    const auto it = enum_names_.find(value);
    if (it != enum_names_.end()) {
      result = it->second;
      return true;
    }
    result = "<Invalid Enum Value>";
    return false;
  }

  static bool parse(const std::string& name, EnumT& result) {
    const auto it =
        std::find_if(enum_names_.begin(), enum_names_.end(), [&name](const auto& pair) { return pair.second == name; });
    if (it != enum_names_.end()) {
      result = it->first;
      return true;
    }
    result = EnumT();
    return false;
  }

  // Global data for the enum.
  inline static std::map<EnumT, std::string> enum_names_;
  inline static std::string enum_values_list_;
  inline static std::string enum_names_list_;
  // NOTE(lschmid): This is currently implemented with a global mutex, such that the declared enums can be used across
  // threads. Since config code is typically not called frequently, this should not be a performance issue.
  inline static std::mutex mutex_;
};

}  // namespace config
