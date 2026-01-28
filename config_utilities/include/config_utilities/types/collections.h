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

#include <string>

namespace config {
namespace detail {

template <class FuncT>
struct ConvertArgs;

template <typename OrigT, typename InterT>
struct ConvertArgs<void(InterT, OrigT, std::string&)> {
  using intermediate = std::remove_const_t<std::remove_reference_t<InterT>>;
  using original = std::remove_const_t<std::remove_reference_t<OrigT>>;
};

template <typename Converter>
using intermediate_t = typename ConvertArgs<decltype(Converter::fromIntermediate)>::intermediate;

template <typename Converter>
using original_t = typename ConvertArgs<decltype(Converter::fromIntermediate)>::original;

}  // namespace detail

/**
 * @brief Field conversion that applies another conversion to keys in a map
 * @tparam Converter Conversion to apply to every key in the map
 *
 * Automatic type inference *should* work as long as the conversion is specified, e.g.,
 * `field<MapKeyConverter<ConverterToApply>>(config.field, "field");`
 * Can be used for any "associative" collection type that exposes a forward iterator and `emplace()`, e.g., `std::map`
 * or `std::unordered_map`.
 *
 * Note that keys that result in a conversion error will be dropped
 */
template <typename Converter>
struct MapKeyConverter {
  using InterT = detail::intermediate_t<Converter>;
  using KeyT = detail::original_t<Converter>;

  template <typename ValueT, template <typename K, typename V> typename Map>
  static Map<InterT, ValueT> toIntermediate(const Map<KeyT, ValueT> original, std::string& error) {
    Map<InterT, ValueT> intermediate;
    for (const auto& [key, value] : original) {
      std::string key_error;
      auto new_key = Converter::toIntermediate(key, key_error);
      if (!key_error.empty()) {
        error += " key conversion failure: " + key_error;
        continue;
      }

      intermediate.emplace(new_key, value);
    }

    return intermediate;
  }

  template <typename ValueT, template <typename K, typename V> typename Map>
  static void fromIntermediate(const Map<InterT, ValueT>& intermediate,
                               Map<InterT, ValueT>& original,
                               std::string& error) {
    original.clear();
    for (const auto& [key, value] : intermediate) {
      std::string key_error;
      KeyT new_key;
      Converter::fromIntermediate(key, new_key, key_error);
      if (!key_error.empty()) {
        error += " key conversion failure: " + key_error;
        continue;
      }

      original.emplace(new_key, value);
    }
  }
};

/**
 * @brief Field conversion that applies another conversion to values in a map
 * @tparam Converter Conversion to apply to every value in the map
 *
 * Automatic type inference *should* work as long as the conversion is specified, e.g.,
 * `field<MapValueConverter<ConverterToApply>>(config.field, "field");`
 * Can be used for any "associative" collection type that exposes a forward iterator and `emplace()`, e.g., `std::map`
 * or `std::unordered_map`.
 *
 * Note that values that result in a conversion error will be dropped
 */
template <typename Converter>
struct MapValueConverter {
  using InterT = detail::intermediate_t<Converter>;
  using ValueT = detail::original_t<Converter>;

  template <typename KeyT, template <typename K, typename V> typename Map>
  static Map<KeyT, InterT> toIntermediate(const Map<KeyT, ValueT> original, std::string& error) {
    Map<KeyT, InterT> intermediate;
    for (const auto& [key, value] : original) {
      std::string value_error;
      auto new_value = Converter::toIntermediate(value, value_error);
      if (!value_error.empty()) {
        error += " value conversion failure: " + value_error;
        continue;
      }

      intermediate.emplace(key, new_value);
    }

    return intermediate;
  }

  template <typename KeyT, template <typename K, typename V> typename Map>
  static void fromIntermediate(const Map<KeyT, InterT>& intermediate, Map<KeyT, ValueT>& original, std::string& error) {
    original.clear();
    for (const auto& [key, value] : intermediate) {
      std::string value_error;
      ValueT new_value;
      Converter::fromIntermediate(value, new_value, value_error);
      if (!value_error.empty()) {
        error += " value conversion failure: " + value_error;
        continue;
      }

      original.emplace(key, new_value);
    }
  }
};

/**
 * @brief Field conversion that applies another conversion to values in a sequence
 * @tparam Converter Conversion to apply to every value in the sequence
 *
 * Automatic type inference *should* work as long as the conversion is specified, e.g.,
 * `field<SequenceConverter<ConverterToApply>>(config.field, "field");`
 * Can be used for any collection type that exposes a forward iterator and `emplace_back()`, e.g., `std::vector` or
 * `std::list`.
 *
 * Note that values that result in a conversion error will be dropped
 */
template <typename Converter>
struct SequenceConverter {
  using OrigT = detail::original_t<Converter>;
  using InterT = detail::intermediate_t<Converter>;

  template <template <typename T> typename Seq>
  static Seq<InterT> toIntermediate(const Seq<OrigT> original, std::string& error) {
    Seq<InterT> intermediate;
    for (const auto& orig : original) {
      std::string value_error;
      auto new_value = Converter::toIntermediate(orig, value_error);
      if (!value_error.empty()) {
        error += " value conversion failure: " + value_error;
        continue;
      }

      intermediate.emplace_back(new_value);
    }

    return intermediate;
  }

  template <template <typename T> typename Seq>
  static void fromIntermediate(const Seq<InterT>& intermediate, Seq<OrigT>& original, std::string& error) {
    original.clear();
    for (const auto& value : intermediate) {
      std::string value_error;
      OrigT new_value;
      Converter::fromIntermediate(value, new_value, value_error);
      if (!value_error.empty()) {
        error += " value conversion failure: " + value_error;
        continue;
      }

      original.emplace_back(new_value);
    }
  }
};

}  // namespace config
