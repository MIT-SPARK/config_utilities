#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

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
  explicit YamlParser(const YAML::Node& node) : node_(node) {}
  ~YamlParser() = default;

  // Access tools.
  std::vector<std::string> children() const;
  const YAML::Node& node() const { return node_; }
  YAML::Node& node() { return node_; }
  const std::vector<std::string>& errors() const { return errors_; }

  /**
   * @brief Parse a value from the yaml node. If the value is not found, the value is not modified, and thus should
   * remain the default value. If the value is found, but the conversion fails, a warning is issued and the value is
   * not modified.
   *
   * @tparam T Type of the value to parse.
   * @param name Name of the param to look up.
   * @param value Value to parse.
   * @return true If the value was found and successfully parsed.
   */
  template <typename T>
  bool fromYaml(const std::string& name, T& value) {
    YAML::Node child_node = node_[name];
    if (!child_node) {
      // The param is not defined. This is not an error.
      return false;
    }
    std::string error;
    try {
      error = fromYamlImpl(value, child_node);
    } catch (const std::exception& e) {
      error = e.what();
    }
    if (error.empty()) {
      return true;
    }
    errors_.emplace_back("Failed to parse param '" + name + "': " + error);
    return false;
  }

  /**
   * @brief Parse a C++ value to the yaml node. If the conversion fails, a warning is issued and the node is not
   * modified.
   *
   * @tparam T Type of the value to parse.
   * @param name Name of the param to store.
   * @param value Value to parse.
   * @return true If the value was successfully parsed.
   */
  template <typename T>
  bool toYaml(const std::string& name, const T& value) {
    std::string error;
    try {
      error = toYamlImpl(name, value);
    } catch (const std::exception& e) {
      error = e.what();
    }
    if (error.empty()) {
      return true;
    }
    errors_.emplace_back("Failed to parse param '" + name + "': " + error);
    return false;
  }

 private:
  // Specializations for parsing different types. These add error messages if the parsing fails.

  // Not config enums.
  template <typename T, typename std::enable_if<std::negation<is_config_enum<T>>::value, bool>::type = true>
  std::string fromYamlImpl(T& value, const YAML::Node& node) const {
    value = node.as<T>();
    return std::string();
  }
  template <typename T, typename std::enable_if<std::negation<is_config_enum<T>>::value, bool>::type = true>
  std::string toYamlImpl(const std::string& name, const T& value) {
    node_[name] = value;
    return std::string();
  }

  // Config enums. TODO(lschmid): Double check and verify how DECLARE_CONFIG_ENUM is used.
  template <typename T, typename std::enable_if<is_config_enum<T>::value, bool>::type = true>
  std::string fromYamlImpl(T& value, const YAML::Node& node) const {
    const auto placeholder = node_.as<std::string>();
    readConfigEnumFromString(placeholder, value);
    return std::string();
  }
  template <typename T, typename std::enable_if<is_config_enum<T>::value, bool>::type = true>
  std::string toYamlImpl(const std::string& name, const T& value) {
    node_[name] = configEnumToString(value);
    return std::string();
  }

  // TODO(lschmid): Add support for sub-configs.

  // Vector.
  template <typename T>
  std::string fromYamlImpl(std::vector<T>& value, const YAML::Node& node) const {
    if (!node_.IsSequence()) {
      return "Data is not a sequence.";
    }
    value = node_.as<std::vector<T>>();
    return std::string();
  }
  template <typename T>
  std::string toYamlImpl(const std::string& name, const std::vector<T>& value) {
    for (const T& element : value) {
      node_[name].push_back(element);
    }
    return std::string();
  }

  // Set.
  template <typename T>
  std::string fromYamlImpl(std::set<T>& value, const YAML::Node& node) const {
    if (!node_.IsSequence()) {
      return "Data is not a sequence.";
    }
    std::vector<T> placeholder = node_.as<std::vector<T>>();
    value.clear();
    value.insert(placeholder.begin(), placeholder.end());
    return std::string();
  }
  template <typename T>
  std::string toYamlImpl(const std::string& name, const std::set<T>& value) {
    for (const T& element : value) {
      node_[name].push_back(element);
    }
    return std::string();
  }

  template <typename K, typename V>
  std::string fromYamlImpl(std::map<K, V>& value, const YAML::Node& node) const {
    if (!node_.IsMap()) {
      return "Data is not a map.";
    }
    value = node_.as<std::map<K, V>>();
    return std::string();
  }
  template <typename K, typename V>
  std::string toYamlImpl(const std::string& name, const std::map<K, V>& value) {
    for (const auto& kv_pair : value) {
      node_[name][kv_pair.first] = kv_pair.second;
    }
    return std::string();
  }

  std::string fromYamlImpl(uint8_t& value, const YAML::Node& node) const;
  std::string toYamlImpl(const std::string& name, const uint8_t& value);

  // Members.
  YAML::Node node_;
  std::vector<std::string> errors_;
};

}  // namespace config::internal