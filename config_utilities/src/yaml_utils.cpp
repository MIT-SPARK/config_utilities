
#include "config_utilities/internal/yaml_utils.h"

#include "config_utilities/internal/string_utils.h"

namespace config::internal {

void mergeYamlNodes(YAML::Node& a, const YAML::Node& b) {
  if (!b.IsMap()) {
    // If b is not a map, merge result is b, unless b is null.
    if (b.IsNull() || !b.IsDefined()) {
      return;
    }
    a = YAML::Clone(b);
    return;
  }
  if (!a.IsMap()) {
    // If a is not a map, merge result is b
    a = YAML::Clone(b);
    return;
  }
  if (!b.size()) {
    // If a is a map, and b is an empty map, return a
    return;
  }

  // Merge all entries of b into a.
  for (auto kv_pair : b) {
    if (kv_pair.first.IsScalar()) {
      const std::string& key = kv_pair.first.Scalar();
      if (a[key]) {
        // Node exists. Merge recursively.
        YAML::Node a_sub = a[key];  // This node is a ref.
        mergeYamlNodes(a_sub, kv_pair.second);
      } else {
        a[key] = YAML::Clone(kv_pair.second);
      }
    }
  }
}

YAML::Node lookupNamespace(const YAML::Node& node, const std::string& name_space, const std::string& separator) {
  YAML::Node current_node = node;
  for (const std::string& ns : splitNamespace(name_space, separator)) {
    if (!current_node[ns]) {
      // Full namespace does not exist, make sure to not modify the input node.
      return YAML::Node(YAML::NodeType::Null);
    }
    current_node = current_node[ns];
  }
  return current_node;
}

void moveDownNamespace(YAML::Node& node, const std::string& name_space, const std::string& separator) {
  const std::vector<std::string> namespaces = splitNamespace(name_space, separator);
  for (auto it = namespaces.rbegin(); it != namespaces.rend(); ++it) {
    YAML::Node tmp;   // This is a new node.
    tmp[*it] = node;  // Add the current node as a child.
    node = tmp;
  }
}

}  // namespace config::internal
