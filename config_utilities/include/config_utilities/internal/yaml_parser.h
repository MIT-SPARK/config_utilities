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
  const YAML::Node& getNode() const { return root_node_; }
  void setNode(const YAML::Node& node) { node_ = node; }
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
      fromYamlImpl(value, child_node, error);
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
      toYamlImpl(name, value, error);
    } catch (const std::exception& e) {
      error = std::string(e.what()) + ".";
    }

    if (error.empty()) {
      moveDownNamespace(node_, sub_namespace);
      mergeYamlNodes(root_node_, node_);
      return true;
    }
    errors_.emplace_back("Failed to parse param '" + name_prefix + name + "': " + error);
    return false;
  }

 private:
  // Generic types.
  template <typename T>
  void fromYamlImpl(T& value, const YAML::Node& node, std::string& error) const {
    value = node.as<T>();
  }

  template <typename T>
  void toYamlImpl(const std::string& name, const T& value, std::string& error) {
    node_[name] = value;
  }

  // Specializations for parsing different types. These add error messages if the parsing fails.
  // Vector.
  template <typename T>
  void fromYamlImpl(std::vector<T>& value, const YAML::Node& node, std::string& error) const {
    if (!node.IsSequence()) {
      error = "Data is not a sequence.";
      return;
    }
    value = node.as<std::vector<T>>();
  }

  template <typename T>
  void toYamlImpl(const std::string& name, const std::vector<T>& value, std::string& error) {
    for (const T& element : value) {
      node_[name].push_back(element);
    }
  }

  // Set.
  template <typename T>
  void fromYamlImpl(std::set<T>& value, const YAML::Node& node, std::string& error) const {
    if (!node.IsSequence()) {
      error = "Data is not a sequence.";
      return;
    }
    const std::vector<T> placeholder = node.as<std::vector<T>>();
    value.clear();
    value.insert(placeholder.begin(), placeholder.end());
  }

  template <typename T>
  void toYamlImpl(const std::string& name, const std::set<T>& value, std::string& error) {
    for (const T& element : value) {
      node_[name].push_back(element);
    }
  }

  // Map.
  template <typename K, typename V>
  void fromYamlImpl(std::map<K, V>& value, const YAML::Node& node, std::string& error) const {
    if (!node.IsMap()) {
      error = "Data is not a map.";
      return;
    }
    value = node.as<std::map<K, V>>();
  }

  template <typename K, typename V>
  void toYamlImpl(const std::string& name, const std::map<K, V>& value, std::string& error) {
    for (const auto& kv_pair : value) {
      node_[name][kv_pair.first] = kv_pair.second;
    }
  }

  // uint8
  void fromYamlImpl(uint8_t& value, const YAML::Node& node, std::string& error) const;
  void toYamlImpl(const std::string& name, const uint8_t& value, std::string& error);

  // Members.
  YAML::Node root_node_;  // Data storage.
  YAML::Node node_;       // Node to write to.
  std::vector<std::string> errors_;
};

}  // namespace config::internal
