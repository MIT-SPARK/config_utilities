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

#include <memory>
#include <vector>

#include <config_utilities/dynamic_config.h>
#include <config_utilities_msgs/srv/set_config.hpp>
#include <rclcpp/node_interfaces/get_node_base_interface.hpp>
#include <rclcpp/node_interfaces/get_node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/get_node_services_interface.hpp>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>
#include <std_msgs/msg/string.hpp>

namespace config {

/**
 * @brief Dynamic config server that allows to set and get configs via ROS topics.
 */
class RosDynamicConfigServer {
 public:
  template <typename Node>
  explicit RosDynamicConfigServer(Node&& node);

  using Srv = config_utilities_msgs::srv::SetConfig;

 private:
  // Helper that manages the exposure of each config.
  struct ConfigReceiver {
    ConfigReceiver(const DynamicConfigServer::Key& key, RosDynamicConfigServer* server);
    const DynamicConfigServer::Key key;
    RosDynamicConfigServer* const server;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::Service<Srv>::SharedPtr srv;
    void handle_service(const std::shared_ptr<Srv::Request>& request, std::shared_ptr<Srv::Response> response);
  };

  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_;
  std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_;
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_parameters_;
  std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_;
  std::map<DynamicConfigServer::Key, std::unique_ptr<ConfigReceiver>> configs_;
  DynamicConfigServer server_;

  void onRegister(const DynamicConfigServer::Key& key);
  void onDeregister(const DynamicConfigServer::Key& key);
  void onUpdate(const DynamicConfigServer::Key& key, const YAML::Node& data);
  YAML::Node onSet(const DynamicConfigServer::Key& key, const YAML::Node& new_values);
};

template <typename Node>
RosDynamicConfigServer::RosDynamicConfigServer(Node&& node)
    : node_base_(rclcpp::node_interfaces::get_node_base_interface(node)),
      node_topics_(rclcpp::node_interfaces::get_node_topics_interface(node)),
      node_parameters_(rclcpp::node_interfaces::get_node_parameters_interface(node)),
      node_services_(rclcpp::node_interfaces::get_node_services_interface(node)) {
  // Setup all currently registered configs.
  for (const auto& key : server_.registeredConfigs()) {
    onRegister(key);
  }

  // Register the hooks for the dynamic config server.
  DynamicConfigServer::Hooks hooks;
  hooks.onRegister = [this](const DynamicConfigServer::Key& key) { onRegister(key); };
  hooks.onDeregister = [this](const DynamicConfigServer::Key& key) { onDeregister(key); };
  hooks.onUpdate = [this](const DynamicConfigServer::Key& key, const YAML::Node& values) { onUpdate(key, values); };
  server_.setHooks(hooks);
}

}  // namespace config
