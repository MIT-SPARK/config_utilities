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

#include <rclcpp/rclcpp.hpp>

#include "config_utilities/factory.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/parsing/yaml.h"

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

inline YAML::Node rosToYaml(const rclcpp::Node& node) {
  YAML::Node root;

  auto all_params = node.list_parameters({}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);

  for (std::string& name : all_params.names) {
    // TODO(nathan) optional namespace filter

    name = name.erase(0, nh.getNamespace().length());  // Remove the nodehandle's namespace.
    std::vector<std::string> name_parts = splitNamespace(name);
    std::string local_name = "";
    if (!name_parts.empty()) {
      name = name.substr(1);  // Remove the leading slash.
      local_name = name_parts.back();
      name_parts.pop_back();
    }

    const auto sub_namespace = joinNamespace(name_parts);

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
    mergeYamlNodes(root, local_node);
  }

  return root;
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

}  // namespace config
