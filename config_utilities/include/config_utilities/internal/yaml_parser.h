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

  // Access tools.
  const YAML::Node& getNode() const { return root_node_; }
  void setNode(const YAML::Node& node) { root_node_ = node; }
  const std::vector<std::string>& getErrors() const { return errors_; }
  void resetErrors() { errors_.clear(); }

  /**
   * @brief Parse a value from the yaml node. If the value is not found, the value is not modified, and thus should
   * remain the default value. If the value is found, but the conversion fails, a warning is issued and the value is
   * not modified.
   *
   * @tparam T Type of the value to parse.
   * @param name Name of the param to look up.
   * @param value Value to parse.
   * @param sub_namespace Sub-namespace of the param to look up.
   * @param field_name_prefix Name prefix of the param used only for error logging.
   * @return true If the value was found and successfully parsed.
   */
  template <typename T>
  bool fromYaml(const std::string& name,
                T& value,
                const std::string& sub_namespace,
                const std::string& field_name_prefix) {
    YAML::Node child_node = lookupNamespace(root_node_, sub_namespace + "/" + name);
    if (!child_node) {
      // The param is not defined. This is not an error.
      return false;
    }
    std::string error;
    try {
      fromYamlImpl(value, child_node, error);
    } catch (const std::exception& e) {
      error = std::string(e.what()) + ".";
    }
    if (error.empty()) {
      return true;
    }
    errors_.emplace_back("Failed to parse param '" + field_name_prefix + name + "': " + error);
    return false;
  }

  /**
   * @brief Parse a C++ value to the yaml node. If the conversion fails, a warning is issued and the node is not
   * modified.
   *
   * @tparam T Type of the value to parse.
   * @param name Name of the param to store.
   * @param value Value to parse.
   * @param sub_namespace Sub-namespace of the param to look parse.
   * @param field_name_prefix Name prefix of the param used only for error logging.
   * @return true If the value was successfully parsed.
   */
  template <typename T>
  bool toYaml(const std::string& name,
              const T& value,
              const std::string& sub_namespace,
              const std::string& field_name_prefix) {
    YAML::Node node;
    std::string error;
    try {
      node = toYamlImpl(name, value, error);
    } catch (const std::exception& e) {
      error = std::string(e.what()) + ".";
    }

    if (error.empty()) {
      moveDownNamespace(node, sub_namespace);
      mergeYamlNodes(root_node_, node);
      return true;
    }
    errors_.emplace_back("Failed to parse param '" + field_name_prefix + name + "': " + error);
    return false;
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
      error = "Data is not a sequence.";
      return;
    }
    value = node.as<std::vector<T>>();
  }

  template <typename T>
  static YAML::Node toYamlImpl(const std::string& name, const std::vector<T>& value, std::string& error) {
    YAML::Node node;
    for (const T& element : value) {
      node[name].push_back(element);
    }
    return node;
  }

  // Set.
  template <typename T>
  static void fromYamlImpl(std::set<T>& value, const YAML::Node& node, std::string& error) {
    if (!node.IsSequence()) {
      error = "Data is not a sequence.";
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
    for (const T& element : value) {
      node[name].push_back(element);
    }
    return node;
  }

  // Map.
  template <typename K, typename V>
  static void fromYamlImpl(std::map<K, V>& value, const YAML::Node& node, std::string& error) {
    if (!node.IsMap()) {
      error = "Data is not a map.";
      return;
    }
    value = node.as<std::map<K, V>>();
  }

  template <typename K, typename V>
  static YAML::Node toYamlImpl(const std::string& name, const std::map<K, V>& value, std::string& error) {
    YAML::Node node;
    for (const auto& kv_pair : value) {
      node[name][kv_pair.first] = kv_pair.second;
    }
    return node;
  }

  // Verify data overflow.
  template <typename T, typename std::enable_if<is_int<T>, bool>::type = true>
  static bool checkIntRange(const T& value, const YAML::Node& node, std::string& error) {
    const auto min = node.as<int64_t>();
    const uint64_t max = min < 0 ? min : node.as<uint64_t>();
    if (max > static_cast<uint64_t>(std::numeric_limits<T>::max())) {
      std::stringstream ss;
      ss << "Value '" << max << "' overflows storage max of '" << std::numeric_limits<T>::max() << "'.";
      error = ss.str();
      return false;
    }
    if (min < static_cast<int64_t>(std::numeric_limits<T>::lowest())) {
      std::stringstream ss;
      ss << "Value '" << min << "' underflows storage min of '" << std::numeric_limits<T>::lowest() << "'.";
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
      ss << "Value '" << long_value << "' overflows storage max of '" << std::numeric_limits<T>::max() << "'.";
      error = ss.str();
      return false;
    }
    if (long_value < static_cast<long double>(std::numeric_limits<T>::lowest())) {
      std::stringstream ss;
      ss << "Value '" << long_value << "' underflows storage min of '" << std::numeric_limits<T>::lowest() << "'.";
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

  // Members.
  YAML::Node root_node_;  // Data storage.
  std::vector<std::string> errors_;
};

}  // namespace config::internal
