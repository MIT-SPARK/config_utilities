#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

namespace config::internal {

/**
 * @brief Returns a new node where node b is mergd into a, overwriting values previously defined in a if they can not be
 * merged.
 */
YAML::Node mergeYamlNodes(const YAML::Node& a, const YAML::Node& b);

/**
 * @brief Get the final node of the specified namespace, where each map in the yaml is separated by the separator.
 */
YAML::Node lookupNamespace(const YAML::Node& node, const std::string& name_space, const std::string& separator = "/");

/**
 * @brief Move the node down the specified namespace, where each namespace separated by the separator is represented as
 * a map key.
 */
YAML::Node moveDownNamespace(YAML::Node& node, const std::string& name_space, const std::string& separator = "/");

}  // namespace config::internal
