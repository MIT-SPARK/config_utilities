#pragma once

#include <type_traits>

namespace config {
namespace internal {

// Check for existence of the function declare_config(T&) via SFINAE.
template <typename T, typename = void>
struct is_config_impl : std::false_type {};

template <typename T>
struct is_config_impl<T, std::void_t<decltype(declare_config(std::declval<T&>()))>> : std::true_type {};

}  // namespace internal

/**
 * @brief Check whether a function declare_config(T&) is implemented for a struct T.
 */
template <class T>
inline bool isConfig() {
  return internal::is_config_impl<T>::value;
}

}  // namespace config