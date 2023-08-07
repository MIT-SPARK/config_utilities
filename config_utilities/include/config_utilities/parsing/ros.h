#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <ros/node_handle.h>

#include "config_utilities/factory.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/parsing/yaml.h"  // NOTE(lschmid): This pulls in more than needed buyt avoids code duplication.

namespace config {

namespace internal {

// Translate the parameter server to yaml code. This is inlined to shift the dependency on ROS to the downstream cpp
// file.
inline YAML::Node xmlRpcToYaml(const XmlRpc::XmlRpcValue& xml) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean:
      return YAML::Node(static_cast<bool>(xml));
    case XmlRpc::XmlRpcValue::Type::TypeInt:
      return YAML::Node(static_cast<int>(xml));
    case XmlRpc::XmlRpcValue::Type::TypeDouble:
      return YAML::Node(static_cast<double>(xml));
    case XmlRpc::XmlRpcValue::Type::TypeString:
      return YAML::Node(static_cast<std::string>(xml));
    case XmlRpc::XmlRpcValue::Type::TypeArray: {
      YAML::Node node(YAML::NodeType::Sequence);
      for (int i = 0; i < xml.size(); ++i) {
        node.push_back(xmlRpcToYaml(xml[i]));
      }
      return node;
    }
    case XmlRpc::XmlRpcValue::Type::TypeStruct: {
      YAML::Node node(YAML::NodeType::Map);
      for (auto it = xml.begin(); it != xml.end(); ++it) {
        node[it->first] = xmlRpcToYaml(it->second);
      }
      return node;
    }
    default:
      return YAML::Node();
  }
}

inline YAML::Node rosToYaml(const ros::NodeHandle& nh) {
  YAML::Node node;
  std::vector<std::string> names;
  nh.getParamNames(names);
  for (std::string& name : names) {
    // Resolve namespace.
    if (name.find(nh.getNamespace()) != 0) {
      continue;
    }
    name = name.erase(0, nh.getNamespace().length() + 1);  // Remove the nodehandle's namespace.
    std::vector<std::string> name_parts = splitNamespace(name);
    std::string local_name = name_parts.back();
    name_parts.pop_back();
    std::string sub_namespace = joinNamespace(name_parts);

    // Get the Xml Value
    XmlRpc::XmlRpcValue value;
    nh.getParam(name, value);

    // Convert data to yaml.
    YAML::Node local_node;
    local_node[local_name] = xmlRpcToYaml(value);
    moveDownNamespace(local_node, sub_namespace);
    mergeYamlNodes(node, local_node);
  }
  return node;
}

}  // namespace internal

/**
 * @brief Loads a config from a yaml node.
 *
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param nh The ROS nodehandle to create the config from.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 * Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromRos(const ros::NodeHandle& nh, const std::string& name_space = "") {
  const ros::NodeHandle ns_nh = ros::NodeHandle(nh, name_space);
  const YAML::Node node = internal::rosToYaml(ns_nh);
  return internal::fromYamlImpl(node, "", static_cast<ConfigT*>(nullptr));
}

/**
 * @brief Create a derived type object based on a the data stored in a yaml node. All derived types need to be
 * registered to the factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct. They
 * need to implement a config as a public member struct named 'Config' and use the config as the first constructor
 * argument.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
 * different base-entry in the factory.
 * @param nh The ROS nodehandle containing the type identifier as a param and the data to create the config.
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromROS(const ros::NodeHandle& nh, ConstructorArguments... args) {
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(internal::rosToYaml(nh), args...);
}

/**
 * @brief Create a derived type object based on a the data stored in a yaml node. All derived types need to be
 * registered to the factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct. They
 * need to implement a config as a public member struct named 'Config' and use the config as the first constructor
 * argument.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
 * different base-entry in the factory.
 * @param nh The ROS nodehandle containing the type identifier as a param and the data to create the config.
 * @param name_space Optionally specify a name space to create the object from. Separate names with
 * slashes '/'. Example: "my_config/my_sub_config".
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromROSWithNamespace(const ros::NodeHandle& nh,
                                                  const std::string& name_space,
                                                  ConstructorArguments... args) {
  ros::NodeHandle ns_nh = ros::NodeHandle(nh, name_space);
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(internal::rosToYaml(ns_nh), args...);
}

}  // namespace config
