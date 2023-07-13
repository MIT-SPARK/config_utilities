#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/visitor.h"
#include "config_utilities/traits.h"

namespace config {

namespace internal {

// argument-dependent-lookup (ADL) so definitions of 'declare_config()' can be found anywher. See the following:
// - http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4381.html
// - https:://github.com/nlohmann/json/blob/develop/include/nlohmann/adl_serializer.hpp

// adl indirection
struct declare_config_fn {
  template <typename ConfigT>
  constexpr auto operator()(ConfigT& config) const -> decltype(declare_config(config)) {
    return declare_config(config);
  }
};

}  // namespace internal

namespace {

constexpr const auto& declare_config = config::internal::static_const<internal::declare_config_fn>;

}  // namespace

// Public interfaces to declare properties in declare_config.

/**
 * @brief Set the name of a config. This is used for printing and logging.
 */
void name(const std::string& name) { internal::Visitor::visitName(name); }

/**
 * @brief Declare string-named fields of the config. This string will be used to get the configs field values during
 * creation, and for checking of validity.
 *
 * @param field The config member that stores data.
 * @param field_name The name of the field.
 * @param unit Optionally define the unit of the field during printing.
 */
template <typename T>
void field(T& field, const std::string& field_name, const std::string& unit = "") {
  internal::Visitor::visitField(field, field_name, unit);
}

/**
 * @brief Declare an enum to as field of a config, that will be parsed and checked by value names.
 *
 * @tparam EnumT The enum type.
 * @param field The config member that stores data.
 * @param field_name The name of the field.
 * @param enum_names Map of enum values to names for non-sequential enums.
 */
template <typename EnumT>
void enum_field(EnumT& field, const std::string& field_name, const std::map<EnumT, std::string>& enum_names) {
  internal::Visitor::visitEnumField(field, field_name, enum_names);
}

/**
 * @brief Declare an enum to as field of a config, that will be parsed and checked by value names.
 *
 * @tparam EnumT The enum type.
 * @param field The config member that stores data.
 * @param field_name The name of the field.
 * @param enum_names List of all possible enum names in identical order to the enum definition. These will be casted to
 * enum. Use only with sequential enums.
 */
template <typename EnumT>
void enum_field(EnumT& field, const std::string& field_name, const std::vector<std::string>& enum_names) {
  std::map<EnumT, std::string> enum_map;
  for (size_t i = 0; i < enum_names.size(); ++i) {
    enum_map[static_cast<EnumT>(i)] = enum_names[i];
  }
  enum_field(field, field_name, enum_map);
}

/**
 * @brief Declare a struct that is also a config as field.
 *
 * @tparam ConfigT The config type.
 * @param config The config member that stores data.
 * @param field_name The name of the field.
 * @param sub_namespace Optionally define a sub namespace to read parameter values for the subconfig from.
 */
template <typename ConfigT>
void subconfig(ConfigT& config, const std::string& field_name, const std::string& sub_namespace = "") {
  internal::Visitor::visitSubconfig(config, field_name, sub_namespace);
}

/**
 * @brief Declare that this config inherits from a base config.
 *
 * @tparam ConfigT The base config type.
 * @param config The config object that is being declared.
 * @param sub_namespace Optionally define a sub namespace to read parameter values for the base config from.
 */
template <typename ConfigT>
void base(ConfigT& config, const std::string& sub_namespace = "") {
  internal::Visitor::visitBase(config, sub_namespace);
}

/**
 * @brief Execute a greater than (GT) check, i.e. param > value.
 *
 * @tparam T type of the parameter to be checked.
 * @param param Value of the parameter to be compared.
 * @param value Value of the reference to compare to.
 * @param name Name of the parameter to be reported in the error summary.
 */
template <typename T>
void checkGT(const T& param, const T& value, const std::string& name) {
  internal::Visitor::visitCheck(internal::Visitor::CheckMode::kGT, param, value, name);
}

/**
 * @brief Execute a greater equal (GE) check, i.e. param >= value.
 *
 * @tparam T type of the parameter to be checked.
 * @param param Value of the parameter to be compared.
 * @param value Value of the reference to compare to.
 * @param name Name of the parameter to be reported in the error summary.
 */
template <typename T>
void checkGE(const T& param, const T& value, const std::string& name) {
  internal::Visitor::visitCheck(internal::Visitor::CheckMode::kGE, param, value, name);
}

/**
 * @brief Execute a less than (LT) check, i.e. param < value.
 *
 * @tparam T type of the parameter to be checked.
 * @param param Value of the parameter to be compared.
 * @param value Value of the reference to compare to.
 * @param name Name of the parameter to be reported in the error summary.
 */
template <typename T>
void checkLT(const T& param, const T& value, const std::string& name) {
  internal::Visitor::visitCheck(internal::Visitor::CheckMode::kLT, param, value, name);
}

/**
 * @brief Execute a less equal (LE) check, i.e. param <= value.
 *
 * @tparam T type of the parameter to be checked.
 * @param param Value of the parameter to be compared.
 * @param value Value of the reference to compare to.
 * @param name Name of the parameter to be reported in the error summary.
 */
template <typename T>
void checkLE(const T& param, const T& value, const std::string& name) {
  internal::Visitor::visitCheck(internal::Visitor::CheckMode::kLE, param, value, name);
}

/**
 * @brief Execute an equal (EQ) check, i.e. param == value.
 *
 * @tparam T type of the parameter to be checked.
 * @param param Value of the parameter to be compared.
 * @param value Value of the reference to compare to.
 * @param name Name of the parameter to be reported in the error summary.
 */
template <typename T>
void checkEQ(const T& param, const T& value, const std::string& name) {
  internal::Visitor::visitCheck(internal::Visitor::CheckMode::kEQ, param, value, name);
}

/**
 * @brief Execute a not equal (NE) check, i.e. param != value.
 *
 * @tparam T type of the parameter to be checked.
 * @param param Value of the parameter to be compared.
 * @param value Value of the reference to compare to.
 * @param name Name of the parameter to be reported in the error summary.
 */
template <typename T>
void checkNE(const T& param, const T& value, const std::string& name) {
  internal::Visitor::visitCheck(internal::Visitor::CheckMode::kNE, param, value, name);
}

/**
 * @brief Execute a range check, i.e. lower <= param <= higher.
 *
 * @tparam T type of the parameter to be checked.
 * @param param Value of the parameter to be compared.
 * @param lower Lower bound of valid values.
 * @param higher Higher bound of valid values.
 * @param name Name of the parameter to be reported in the error summary.
 */
template <typename T>
void checkInRange(const T& param, const T& lower, const T& higher, const std::string& name) {
  internal::Visitor::visitCheckInRange(param, lower, higher, name);
}

/**
 * @brief Execute a condition check, i.e. whether condition is true.
 *
 * @param condition Condition that should evaluate to true if the config is valid.
 * @param warning Message to be reported in the error summary.
 */
void checkCondition(bool condition, const std::string& warning) {
  internal::Visitor::visitCheckCondition(condition, warning);
}

}  // namespace config
