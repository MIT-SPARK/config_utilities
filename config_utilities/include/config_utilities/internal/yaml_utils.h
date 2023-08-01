#pragma once

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace config::internal {

/**
 * @brief Merges node b into a, overwriting values previously defined in a if they can not be
 * merged. Modifies node a, whereas b is const.
 */
void mergeYamlNodes(YAML::Node& a, const YAML::Node& b);

/**
 * @brief Get a pointer to the final node of the specified namespace if it exists, where each map in the yaml is
 * separated by the separator.
 */
YAML::Node lookupNamespace(const YAML::Node& node, const std::string& name_space, const std::string& separator = "/");

/**
 * @brief Move the node down the specified namespace, where each namespace separated by the separator is represented as
 * a map key.
 */
void moveDownNamespace(YAML::Node& node, const std::string& name_space, const std::string& separator = "/");

/**
 * @brief Check whether two yaml nodes are equal. Note that since since yaml-cpp operator== checks for identity and not
 * equality, scalar values will be compared by string representation.
 */
bool isEqual(const YAML::Node& a, const YAML::Node& b);

/**
 * @brief Convert a yaml node that contains a map or sequence to a list of corresponding nodes.
 * @param node The node to convert.
 * @return The list of nodes. Nodes stored in this struct are references to the original data.
 */
std::vector<YAML::Node> getNodeArray(const YAML::Node& node);

}  // namespace config::internal
