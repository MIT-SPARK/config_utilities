#pragma once

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/checks.h"
#include "config_utilities/internal/namespacing.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/traits.h"

namespace config {

namespace internal {

// argument-dependent-lookup (ADL) so definitions of 'declare_config()' can be found anywhere. Note that
// 'declare_config' needs to be defined in the same namespace as the object it refers to.See the following:
// - http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4381.html
// - https:://github.com/nlohmann/json/blob/develop/include/nlohmann/adl_serializer.hpp

// ADL indirection.
struct declare_config_fn {
  template <typename ConfigT>
  constexpr auto operator()(ConfigT& config) const -> decltype(declare_config(config)) {
    return declare_config(config);
  }
};

namespace {

constexpr const auto& declare_config = static_const<internal::declare_config_fn>;

}  // namespace

}  // namespace internal

// Public interfaces to declare properties in declare_config.

/**
 * @brief Set the name of a config. This is used for printing and logging.
 */
inline void name(const std::string& name) { internal::Visitor::visitName(name); }

/**
 * @brief Declare string-named fields of the config. This string will be used to get the configs field values during
 * creation, and for checking of validity.
 * @param field The config member that stores data.
 * @param field_name The name of the field.
 * @param unit Optionally define the unit of the field during printing.
 */
template <typename T, typename std::enable_if<!isConfig<T>(), bool>::type = true>
void field(T& field, const std::string& field_name, const std::string& unit = "") {
  internal::Visitor::visitField(field, field_name, unit);
}

/**
 * @brief Declare a field that is a subconfig
 * @param field The config member that stores data.
 * @param field_name The name of the field.
 * @param name_is_namespace Whether or not to enter a new namepsace
 */
template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type = true>
static void field(ConfigT& field, const std::string& field_name, bool name_is_namespace = true) {
  internal::Visitor::visitField(field, field_name, name_is_namespace);
}

/**
 * @brief Declare string-named fields of the config. This string will be used to get the configs field values during
 * creation, and for checking of validity.
 *
 * This version allows for declaring an intermediate conversion for both setting and getting the field.
 *
 * @tparam Conversion Conversion struct. Must declare 'static IntT toIntermediate(T value, std::string& error)' and
 * 'static void fromIntermediate(IntT intermediate, T& value, std::string& error)'.
 * @param field The config member that stores data.
 * @param field_name The name of the field.
 * @param unit Optionally define the unit of the field during printing.
 */
template <typename Conversion, typename T>
void field(T& field, const std::string& field_name, const std::string& unit = "") {
  internal::Visitor::visitField<Conversion>(field, field_name, unit);
}

/**
 * @brief Declare that this config inherits from a base config. Note that this call typically requires explicit
 * template declaration or explicit casting for argument dependent look-up. E.g. 'base<BaseT>(config)' or '
 * base(static_cast<BaseT&>(config))'.
 *
 * @tparam ConfigT The base config type.
 * @param config The config object that is being declared.
 */
template <typename ConfigT>
void base(ConfigT& config) {
  internal::Visitor::visitBase(config);
}

/**
 * @brief Execute a binary comparison check between the param and the value
 *
 * For example, if you wanted to validate that a param is greater than some value, you
 * would use the check `checkBinary<std::greater>(...)`. Note that this casts the value
 * to the same type as the parameter, so there may be some loss of precision in certain
 * cases.
 *
 * @tparam Compare Binary comparison functor to use
 * @tparam T Type of the parameter to be checked (inferred)
 * @tparam P Type of the value to check against (inferred)
 * @param param Value of the parameter to be checked
 * @param value Value to check against
 * @param name Name of the parameter to be reported in warning
 */
template <typename Compare, typename T, typename P>
void checkBinary(const T& param, const P& value, const std::string& name) {
  internal::Visitor::visitCheck(internal::BinaryCheck<T, Compare>(param, static_cast<T>(value), name));
}

/**
 * @brief Comparison mode
 */
enum class CheckMode {
  GT /**< greater than (>) */,
  GE /**< greater than or equal to (>=) */,
  LT /**< less than (<) */,
  LE /**< less than or equal to (<=) */,
  EQ /** equal to (==) */,
  NE /** not equal to (!=) */
};

// Also expose these in the config namespace.
constexpr CheckMode GT = CheckMode::GT;
constexpr CheckMode GE = CheckMode::GE;
constexpr CheckMode LT = CheckMode::LT;
constexpr CheckMode LE = CheckMode::LE;
constexpr CheckMode EQ = CheckMode::EQ;
constexpr CheckMode NE = CheckMode::NE;

/**
 * @brief Execute a binary comparison check between the param and the value, where the kind of check to be performed is
 * specified by the mode.
 *
 * For example, if you wanted to validate that a param is greater than some value, you
 * would use the check `check(param, config::GT, value, "param")`. Note that this casts the value
 * to the same type as the parameter, so there may be some loss of precision in certain
 * cases.
 *
 * @tparam T Type of the parameter to be checked (inferred).
 * @tparam P Type of the value to check against (inferred).
 * @param param Value of the parameter to be checked.
 * @param mode comparison functor to use.
 * @param value Value to check against.
 * @param name Name of the parameter to be reported in warning.
 */
template <typename T, typename P>
void check(const T& param, CheckMode mode, const P& value, const std::string& name) {
  switch (mode) {
    case CheckMode::GT:
      checkBinary<std::greater<T>>(param, value, name);
      break;
    case CheckMode::GE:
      checkBinary<std::greater_equal<T>>(param, value, name);
      break;
    case CheckMode::LT:
      checkBinary<std::less<T>>(param, value, name);
      break;
    case CheckMode::LE:
      checkBinary<std::less_equal<T>>(param, value, name);
      break;
    case CheckMode::EQ:
      checkBinary<std::equal_to<T>>(param, value, name);
      break;
    case CheckMode::NE:
      checkBinary<std::not_equal_to<T>>(param, value, name);
      break;
  }
}

/**
 * @brief Execute a range check, i.e. lower <= param <= higher.
 *
 * @tparam T type of the parameter to be checked.
 * @param param Value of the parameter to be compared.
 * @param lower Lower bound of valid values.
 * @param higher Higher bound of valid values.
 * @param name Name of the parameter to be reported in the error summary.
 * @param lower_inclusive Whether the lower end of the range range is closed (i.e., [low, high] or open (low, high])
 * @param upper_inclusive Whether the upper end of the range range is closed (i.e., [low, high] or open [low, high))
 */
template <typename T>
void checkInRange(const T& param,
                  const T& lower,
                  const T& higher,
                  const std::string& name,
                  bool lower_inclusive = true,
                  bool upper_inclusive = true) {
  internal::Visitor::visitCheck(internal::CheckRange(param, lower, higher, name, lower_inclusive, upper_inclusive));
}

/**
 * @brief Execute a condition check, i.e. whether condition is true.
 * @param condition Condition that should evaluate to true if the config is valid.
 * @param warning Message to be reported in the error summary.
 */
inline void checkCondition(bool condition, const std::string& warning) {
  internal::Visitor::visitCheck(internal::Check(condition, warning));
}

/**
 * @brief Execute a check whether a parameter is one of a set of candidates.
 * @param param Parameter to check. Must implement operotr==.
 * @param candidates List of candidates to check against.
 * @param name Name of the parameter to be reported in the error summary.
 */
template <typename T>
inline void checkIsOneOf(const T& param, const std::vector<T>& candidates, const std::string& name) {
  internal::Visitor::visitCheck(internal::CheckIsOneOf(param, candidates, name));
}

/**
 * @brief Execute a custom check.
 * @param check Custom check class to validate.
 */
inline void check(const internal::CheckBase& check) { internal::Visitor::visitCheck(check); }

/**
 * @brief Execute a custom check
 * @tparam CheckType Check type to actually perform
 * @param args Arguments to check constructor
 */
template <typename CheckType, typename... Ts>
inline void check(Ts... args) {
  internal::Visitor::visitCheck(CheckType(std::forward<Ts>(args)...));
}

/**
 * @brief Execute a custom check
 * @note Only differs in allowing parameter inference for template check types
 * @tparam CheckType Check type to actually perform
 * @param args Arguments to check constructor
 */
template <template <typename...> typename CheckType, typename... Ts>
inline void check(Ts... args) {
  internal::Visitor::visitCheck(CheckType(std::forward<Ts>(args)...));
}

}  // namespace config
