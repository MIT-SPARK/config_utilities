#pragma once

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
  const YAML::Node& node() const { return root_node_; }
  YAML::Node& node() { return root_node_; }
  const std::vector<std::string>& errors() const { return errors_; }

  /**
   * @brief Parse a value from the yaml node. If the value is not found, the value is not modified, and thus should
   * remain the default value. If the value is found, but the conversion fails, a warning is issued and the value is
   * not modified.
   *
   * @tparam T Type of the value to parse.
   * @param name Name of the param to look up.
   * @param value Value to parse.
   * @param sub_namespace Sub-namespace of the param to look up.
   * @param name_prefix Name prefix of the param used only for error logging.
   * @return true If the value was found and successfully parsed.
   */
  template <typename T>
  bool fromYaml(const std::string& name, T& value, const std::string& sub_namespace, const std::string& name_prefix) {
    YAML::Node child_node = lookupNamespace(root_node_, sub_namespace)[name];

    if (!child_node) {
      // The param is not defined. This is not an error.
      return false;
    }
    std::string error;
    try {
      error = fromYamlImpl(value, child_node);
    } catch (const std::exception& e) {
      error = std::string(e.what()) + ".";
    }
    if (error.empty()) {
      return true;
    }
    errors_.emplace_back("Failed to parse param '" + name_prefix + name + "': " + error);
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
   * @param name_prefix Name prefix of the param used only for error logging.
   * @return true If the value was successfully parsed.
   */
  template <typename T>
  bool toYaml(const std::string& name,
              const T& value,
              const std::string& sub_namespace,
              const std::string& name_prefix) {
    node_ = YAML::Node();
    std::string error;
    try {
      error = toYamlImpl(name, value);
    } catch (const std::exception& e) {
      error = std::string(e.what()) + ".";
    }

    if (error.empty()) {
      root_node_ = mergeYamlNodes(root_node_, moveDownNamespace(node_, sub_namespace));
      return true;
    }
    errors_.emplace_back("Failed to parse param '" + name_prefix + name + "': " + error);
    return false;
  }

 private:
  // Generic types.
  template <typename T>
  std::string fromYamlImpl(T& value, const YAML::Node& node) const {
    value = node.as<T>();
    return std::string();
  }
  template <typename T>
  std::string toYamlImpl(const std::string& name, const T& value) {
    node_[name] = value;
    return std::string();
  }

  // Specializations for parsing different types. These add error messages if the parsing fails.
  // Vector.
  template <typename T>
  std::string fromYamlImpl(std::vector<T>& value, const YAML::Node& node) const {
    if (!node.IsSequence()) {
      return "Data is not a sequence.";
    }
    value = node.as<std::vector<T>>();
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
    if (!node.IsSequence()) {
      return "Data is not a sequence.";
    }
    const std::vector<T> placeholder = node.as<std::vector<T>>();
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
    if (!node.IsMap()) {
      return "Data is not a map.";
    }
    value = node.as<std::map<K, V>>();
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
  YAML::Node root_node_;  // Data storage.
  YAML::Node node_;       // Node to write to.
  std::vector<std::string> errors_;
};

}  // namespace config::internal
