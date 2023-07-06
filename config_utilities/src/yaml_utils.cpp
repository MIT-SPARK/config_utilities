
#include "config_utilities/internal/yaml_utils.h"

#include "config_utilities/internal/string_utils.h"

namespace config::internal {

YAML::Node mergeYamlNodes(const YAML::Node& a, const YAML::Node& b) {
  if (!b.IsMap()) {
    // If b is not a map, merge result is b, unless b is null
    return YAML::Clone((b.IsNull() || !b.IsDefined()) ? a : b);
  }
  if (!a.IsMap()) {
    // If a is not a map, merge result is b
    return YAML::Clone(b);
  }
  if (!b.size()) {
    // If a is a map, and b is an empty map, return a
    return YAML::Clone(a);
  }
  // Create a new map 'c' with the same mappings as a, merged with b
  auto c = YAML::Node(YAML::NodeType::Map);
  for (auto n : a) {
    if (n.first.IsScalar()) {
      const std::string& key = n.first.Scalar();
      auto t = YAML::Node(b[key]);
      if (t) {
        c[n.first] = mergeYamlNodes(n.second, t);
        continue;
      }
    }
    c[n.first] = n.second;
  }
  // Add the mappings from 'b' not already in 'c'
  for (auto n : b) {
    if (!n.first.IsScalar() || !c[n.first.Scalar()]) {
      c[n.first] = n.second;
    }
  }
  return c;
}

YAML::Node lookupNamespace(const YAML::Node& node, const std::string& name_space, const std::string& separator) {
  YAML::Node current_node = node;
  for (const std::string& ns : splitNamespace(name_space, separator)) {
    current_node = current_node[ns];
  }
  return current_node;
}

YAML::Node moveDownNamespace(YAML::Node& node, const std::string& name_space, const std::string& separator) {
  const std::vector<std::string> namespaces = splitNamespace(name_space, separator);
  for (auto it = namespaces.rbegin(); it != namespaces.rend(); ++it) {
    YAML::Node tmp;
    tmp[*it] = node;
    node = tmp;
  }
  return node;
}

}  // namespace config::internal
