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
#include <map>
#include <set>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/traits.h"

namespace config::internal {

/**
 * @brief Since yaml is used as internal data interface, this class is the primary conversion tool from config utilities
 * internal representation (yaml) to the C++ structs and back. Support for additional types can be added by implementing
 * yaml-cpp  conversion structs for the types.
 */
class YamlParser {
 public:
  YamlParser() = default;
  ~YamlParser() = default;

  /**
   * @brief Parse a value from the yaml node. If the value is not found, the value is not modified, and thus should
   * remain the default value. If the value is found, but the conversion fails, a warning is issued and the value is
   * not modified.
   *
   * @tparam T Type of the value to parse.
   * @param node The yaml node to parse the value from.
   * @param name Name of the param to look up.
   * @param value Value to parse.
   * @param sub_namespace Sub-namespace of the param to look up in the node.
   * @param error Where to store the error message if conversion fails.
   * @return true If the value was found and successfully parsed.
   */
  template <typename T>
  static bool fromYaml(const YAML::Node& node,
                       const std::string& name,
                       T& value,
                       const std::string& sub_namespace,
                       std::string& error) {
    YAML::Node child_node = lookupNamespace(node, sub_namespace + "/" + name);
    if (!child_node) {
      // The param is not defined. This is not an error.
      return false;
    }
    try {
      fromYamlImpl(value, child_node, error);
    } catch (const std::exception& e) {
      error = std::string(e.what());
    }
    return error.empty();
  }

  /**
   * @brief Parse a C++ value to the yaml node. If the conversion fails, a warning is issued and the node is not
   * modified.
   *
   * @tparam T Type of the value to parse.
   * @param name Name of the param to store.
   * @param value Value to parse.
   * @param sub_namespace Sub-namespace of the param when adding it to the root node.
   * @param error Where to store the error message if conversion fails.
   * @return The yaml node the value was successfully parsed. Null-node if conversion failed.
   */
  template <typename T>
  static YAML::Node toYaml(const std::string& name,
                           const T& value,
                           const std::string& sub_namespace,
                           std::string& error) {
    YAML::Node node;
    try {
      node = toYamlImpl(name, value, error);
    } catch (const std::exception& e) {
      error = std::string(e.what());
    }

    if (!error.empty()) {
      return YAML::Node(YAML::NodeType::Null);
    }

    moveDownNamespace(node, sub_namespace);
    return node;
  }

 private:
  // Generic types.
  template <typename T,
            typename std::enable_if<!is_int<T>, bool>::type = true,
            typename std::enable_if<!std::is_floating_point<T>::value, bool>::type = true>
  static void fromYamlImpl(T& value, const YAML::Node& node, std::string& error) {
    value = node.as<T>();
  }

  template <typename T>
  static YAML::Node toYamlImpl(const std::string& name, const T& value, std::string& error) {
    YAML::Node node;
    node[name] = value;
    return node;
  }

  // Specializations for parsing different types. These add error messages if the parsing fails.
  // Vector.
  template <typename T>
  static void fromYamlImpl(std::vector<T>& value, const YAML::Node& node, std::string& error) {
    if (!node.IsSequence()) {
      error = "Data is not a sequence";
      return;
    }
    value = node.as<std::vector<T>>();
  }

  template <typename T>
  static YAML::Node toYamlImpl(const std::string& name, const std::vector<T>& value, std::string& error) {
    YAML::Node node;
    node[name] = YAML::Node(YAML::NodeType::Sequence);
    for (const T& element : value) {
      node[name].push_back(element);
    }
    return node;
  }

  // Set.
  template <typename T>
  static void fromYamlImpl(std::set<T>& value, const YAML::Node& node, std::string& error) {
    if (!node.IsSequence()) {
      error = "Data is not a sequence";
      return;
    }
    std::set<std::string> repeated_entries;
    value.clear();
    for (const auto& element : node) {
      const T& element_value = element.as<T>();
      if (value.find(element_value) != value.end()) {
        repeated_entries.insert(dataToString(element));
      } else {
        value.insert(element_value);
      }
    }
    if (!repeated_entries.empty()) {
      auto it = repeated_entries.begin();
      error = "Repeated entries '" + *it;
      while (++it != repeated_entries.end()) {
        error += "', '" + *it;
      }
      error += "'.";
    }
  }

  template <typename T>
  static YAML::Node toYamlImpl(const std::string& name, const std::set<T>& value, std::string& error) {
    YAML::Node node;
    node[name] = YAML::Node(YAML::NodeType::Sequence);
    for (const T& element : value) {
      node[name].push_back(element);
    }

    return node;
  }

  // Map.
  template <typename K, typename V>
  static void fromYamlImpl(std::map<K, V>& value, const YAML::Node& node, std::string& error) {
    if (!node.IsMap()) {
      error = "Data is not a map";
      return;
    }
    value = node.as<std::map<K, V>>();
  }

  template <typename K, typename V>
  static YAML::Node toYamlImpl(const std::string& name, const std::map<K, V>& value, std::string& error) {
    YAML::Node node;
    node[name] = YAML::Node(YAML::NodeType::Map);
    for (const auto& kv_pair : value) {
      node[name][kv_pair.first] = kv_pair.second;
    }
    return node;
  }

  // Verify data overflow.
  template <typename T, typename std::enable_if<is_int<T>, bool>::type = true>
  static bool checkIntRange(const T& value, const YAML::Node& node, std::string& error) {
    // NOTE(lschmid): We assume we don't get integers larger than 64 bit. Also bool, uchar, and string are checked
    // separately.
    const int64_t min = node.as<int64_t>();
    // note: usigned parsing fails when number is explicitly negative. defaulting to 0 in these cases is the correct
    // behavior: the number is explicilty signed and negative, and can only overflow via min
    const uint64_t max = node.as<uint64_t>(0);
    if (min > 0 && max > static_cast<uint64_t>(std::numeric_limits<T>::max())) {
      std::stringstream ss;
      ss << "Value '" << max << "' overflows storage max of '" << std::numeric_limits<T>::max() << "'";
      error = ss.str();
      return false;
    }
    if (min < static_cast<int64_t>(std::numeric_limits<T>::lowest())) {
      std::stringstream ss;
      ss << "Value '" << min << "' underflows storage min of '" << std::numeric_limits<T>::lowest() << "'";
      error = ss.str();
      return false;
    }
    return true;
  }

  template <typename T, typename std::enable_if<std::is_floating_point<T>::value, bool>::type = true>
  static bool checkFloatRange(const T& value, const YAML::Node& node, std::string& error) {
    const auto long_value = node.as<long double>();
    if (long_value > static_cast<long double>(std::numeric_limits<T>::max())) {
      std::stringstream ss;
      ss << "Value '" << long_value << "' overflows storage max of '" << std::numeric_limits<T>::max() << "'";
      error = ss.str();
      return false;
    }
    if (long_value < static_cast<long double>(std::numeric_limits<T>::lowest())) {
      std::stringstream ss;
      ss << "Value '" << long_value << "' underflows storage min of '" << std::numeric_limits<T>::lowest() << "'";
      error = ss.str();
      return false;
    }
    return true;
  }

  // Specializations for integral types.
  template <typename T,
            typename std::enable_if<is_int<T>, bool>::type = true,
            typename std::enable_if<!std::is_floating_point<T>::value, bool>::type = true>
  static void fromYamlImpl(T& value, const YAML::Node& node, std::string& error) {
    if (!checkIntRange(value, node, error)) {
      return;
    }
    value = node.as<T>();
  }

  // Specializations for float types.
  template <typename T,
            typename std::enable_if<!is_int<T>, bool>::type = true,
            typename std::enable_if<std::is_floating_point<T>::value, bool>::type = true>
  static void fromYamlImpl(T& value, const YAML::Node& node, std::string& error) {
    if (!checkFloatRange(value, node, error)) {
      return;
    }
    value = node.as<T>();
  }

  // Specialization for uint8 to not represent it as char but as number.
  static void fromYamlImpl(uint8_t& value, const YAML::Node& node, std::string& error);
  static YAML::Node toYamlImpl(const std::string& name, const uint8_t& value, std::string& error);
};

}  // namespace config::internal
