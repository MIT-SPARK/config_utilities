#pragma once

#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/meta_data.h"
#include "config_utilities/traits.h"

namespace config {

namespace internal {

// ADL Indirection so definitions of 'declare_config()' can be found anywhere via ADL.

// adl indirection
struct declare_fn {
  template <typename ConfigT>
  constexpr auto operator()(ConfigT& config) const -> decltype(declare_config(config)) {
    return declare_config(config);
  }
};

// ODR workaround
template <class T>
constexpr T static_const{};

}  // namespace internal

namespace {

constexpr const auto& declare_config = config::internal::static_const<internal::declare_fn>;

}  // namespace

// Properties to declare_config of a config.

/**
 * @brief Set the name of a config. This is used for printing and logging.
 */
void name(const std::string& name) {
  internal::MetaData& data = internal::MetaData::instance();
  data.name = name;
};

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
//   internal::MetaData& data = internal::MetaData::instance();
};

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
  internal::MetaData& data = internal::MetaData::instance();
  if (data.mode == internal::MetaData::Mode::kCheckValid) {
    data.validity_checker.checkGT(param, value, name);
  }
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
  internal::MetaData& data = internal::MetaData::instance();
  if (data.mode == internal::MetaData::Mode::kCheckValid) {
    data.validity_checker.checkGE(param, value, name);
  }
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
  internal::MetaData& data = internal::MetaData::instance();
  if (data.mode == internal::MetaData::Mode::kCheckValid) {
    data.validity_checker.checkLT(param, value, name);
  }
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
  internal::MetaData& data = internal::MetaData::instance();
  if (data.mode == internal::MetaData::Mode::kCheckValid) {
    data.validity_checker.checkLE(param, value, name);
  }
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
  internal::MetaData& data = internal::MetaData::instance();
  if (data.mode == internal::MetaData::Mode::kCheckValid) {
    data.validity_checker.checkEq(param, value, name);
  }
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
  internal::MetaData& data = internal::MetaData::instance();
  if (data.mode == internal::MetaData::Mode::kCheckValid) {
    data.validity_checker.checkNE(param, value, name);
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
 */
template <typename T>
void checkInRange(const T& param, const T& lower, const T& higher, const std::string& name) {
  internal::MetaData& data = internal::MetaData::instance();
  if (data.mode == internal::MetaData::Mode::kCheckValid) {
    data.validity_checker.checkInRange(param, lower, higher, name);
  }
}

/**
 * @brief Execute a condition check, i.e. whether condition is true.
 *
 * @param condition Condition that should evaluate to true if the config is valid.
 * @param warning Message to be reported in the error summary.
 */
void checkCondition(bool condition, const std::string& warning) {
  internal::MetaData& data = internal::MetaData::instance();
  if (data.mode == internal::MetaData::Mode::kCheckValid) {
    data.validity_checker.checkCondition(condition, warning);
  }
}

}  // namespace config