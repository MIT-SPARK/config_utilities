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
#include "config_utilities/update.h"

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
    name = name.erase(0, nh.getNamespace().length());  // Remove the nodehandle's namespace.
    std::vector<std::string> name_parts = splitNamespace(name);
    std::string local_name = "";
    if (!name_parts.empty()) {
      name = name.substr(1);  // Remove the leading slash.
      local_name = name_parts.back();
      name_parts.pop_back();
    }
    const std::string sub_namespace = joinNamespace(name_parts);

    // Get the Xml Value
    XmlRpc::XmlRpcValue value;
    nh.getParam(name, value);

    // Convert data to yaml.
    YAML::Node local_node;
    if (local_name.empty()) {
      local_node = xmlRpcToYaml(value);
    } else {
      local_node[local_name] = xmlRpcToYaml(value);
    }
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
  return fromYaml<ConfigT>(node, "");
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

/**
 * @brief Update the config with the current parameters in ROS.
 * @note This function will update the field and check the validity of the config afterwards. If the config is invalid,
 * the field will be reset to its original value.
  * @param config The config to update.
  * @param nh The ROS nodehandle to update the config from.
  * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 */
template <typename ConfigT>
bool updateFromRos(ConfigT& config, const ros::NodeHandle& nh, const std::string& name_space = "") {
  const ros::NodeHandle ns_nh = ros::NodeHandle(nh, name_space);
  const YAML::Node node = internal::rosToYaml(ns_nh);
  return updateField(config, node, true, name_space);
}

}  // namespace config
