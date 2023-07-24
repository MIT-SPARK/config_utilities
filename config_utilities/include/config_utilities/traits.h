#pragma once

#include <string>
#include <type_traits>

namespace config {
namespace internal {

// Check for existence of the function declare_config(T&) via SFINAE.
template <typename T, typename = void>
struct is_config_impl : std::false_type {};

template <typename T>
struct is_config_impl<T, std::void_t<decltype(declare_config(std::declval<T&>()))>> : std::true_type {};

// Check whether an object is a virtual config.
template <typename T>
struct is_virtual_config : std::false_type {};

// ODR workaround
template <class T>
constexpr T static_const{};

// Define which types are considered ints and floats to check overflow. Define these explicitly as we don't want
// std::is_integral to account for bools.
template <typename T>
constexpr bool is_int = std::is_integral<T>::value && !std::is_same<T, bool>::value;

}  // namespace internal

/**
 * @brief Check whether a function declare_config(T&) is implemented for a struct T.
 */
template <class T>
constexpr bool isConfig() {
  return internal::is_config_impl<T>::value;
}

template <class T>
constexpr bool isConfig(const T& config) {
  return internal::is_config_impl<T>::value;
}

}  // namespace config
