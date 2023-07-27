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
  // Interfaces to for user-side conversion.

  /**
   * @brief Convert an enum value to its string representation.
   * @param value The enum value to be converted.
   * @param print_errors If true, an error message is printed if the value is not a set enum value.
   * @return The string representation of the enum value if the value is in the defined values, '<Invalid Enum
   * Value>' otherwise. This should not happen if all enum values are defined using 'setNames()'.
   */
  static std::string toString(EnumT value, bool print_errors = true) {
    std::string result;
    std::lock_guard<std::mutex> lock(instance().mutex_);
    if (!instance().parse(value, result) && print_errors) {
      internal::Logger::logWarning("Enum conversion failed: Value '" + std::to_string(static_cast<int>(value)) +
                                   "' is out of bounds for EnumT='" + internal::typeName<EnumT>() + "' with values [" +
                                   instance().enum_value_list_ + "].");
    }
    return result;
  }

  /**
   * @brief Convert a enum string representation to its enum value.
   * @param value The string representation of the enum.
   * @param print_errors If true, an error message is printed if the string is not a set enum name.
   * @returns The enum value if the string is in the defined names, the default value of the enum otherwise.
   */
  static EnumT fromString(const std::string& value, bool print_errors = true) {
    auto result = EnumT();
    std::lock_guard<std::mutex> lock(instance().mutex_);
    if (!instance().parse(value, result) && print_errors) {
      internal::Logger::logWarning("Enum conversion failed: Name '" + value + "' is out of bounds for EnumT='" +
                                   internal::typeName<EnumT>() + "' with names [" + instance().enum_name_list_ + "].");
    }
    return result;
  }

  // Interfaces to set the enum names for conversion.

  /**
   * @brief Define the names of the enum values. This is used for conversion between enum values and strings. This
   * function overwrites all previous enum name definitions. The complete list of enum names should be defined before
   * any enums are parsed.
   * @param enum_names Map of enum values to their string names.
   */
  static void setNames(const std::map<EnumT, std::string>& enum_names) {
    std::lock_guard<std::mutex> lock(instance().mutex_);
    instance().enum_names_ = enum_names;
    instance().enum_name_list_ = "";
    instance().enum_value_list_ = "";
    if (!instance().enum_names_.empty()) {
      for (const auto& value_name_pair : instance().enum_names_) {
        instance().enum_name_list_ += "'" + value_name_pair.second + "', ";
        instance().enum_value_list_ += "'" + std::to_string(static_cast<int>(value_name_pair.first)) + "', ";
      }
      instance().enum_name_list_.erase(instance().enum_name_list_.size() - 2);
      instance().enum_value_list_.erase(instance().enum_value_list_.size() - 2);
    }
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
      error = "Value '" + std::to_string(static_cast<int>(value)) + "' is not defined for enum with values [" +
              instance().enum_value_list_ + "]";
    }
    return result;
  }

  static EnumT fromIntermediate(const std::string& value, std::string& error) {
    auto result = EnumT();
    std::lock_guard<std::mutex> lock(instance().mutex_);
    if (!instance().parse(value, result)) {
      error = "Value '" + value + "' is out of bounds for enum with names [" + instance().enum_name_list_ + "]";
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
  std::string enum_value_list_;
  std::string enum_name_list_;
};

}  // namespace config
