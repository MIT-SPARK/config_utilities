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
#include "config_utilities_ros/ros_dynamic_config_server.h"

#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_service.hpp>

namespace config {

RosDynamicConfigServer::ConfigReceiver::ConfigReceiver(const DynamicConfigServer::Key& key,
                                                       RosDynamicConfigServer* server)
    : key(key), server(server) {
  const auto base_topic = "~/" + key;
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  pub = rclcpp::create_publisher<std_msgs::msg::String>(
      server->node_parameters_, server->node_topics_, base_topic + "/get", qos);
  srv = rclcpp::create_service<Srv>(
      server->node_base_,
      server->node_services_,
      base_topic + "/set",
      std::bind(&ConfigReceiver::handle_service, this, std::placeholders::_1, std::placeholders::_2),
      {},
      nullptr);
}

void RosDynamicConfigServer::ConfigReceiver::handle_service(const std::shared_ptr<Srv::Request>& request,
                                                            std::shared_ptr<Srv::Response> response) {
  response->data = YAML::Dump(server->onSet(key, YAML::Load(request->data)));
}

void RosDynamicConfigServer::onRegister(const DynamicConfigServer::Key& key) {
  configs_.emplace(key, std::make_unique<ConfigReceiver>(key, this));

  // Latch the current state of the config.
  onUpdate(key, server_.getInfo(key));
}

void RosDynamicConfigServer::onDeregister(const DynamicConfigServer::Key& key) { configs_.erase(key); }

void RosDynamicConfigServer::onUpdate(const DynamicConfigServer::Key& key, const YAML::Node& data) {
  const auto it = configs_.find(key);
  if (it == configs_.end()) {
    // Shouldn't happen but better to fail gracefully if people extend this.
    internal::Logger::logWarning("Tried to publish to dynamic config '" + key + "' without existing publisher.");
    return;
  }

  // Publish the new config info.
  std_msgs::msg::String msg;
  msg.data = YAML::Dump(data);
  it->second->pub->publish(msg);
}

YAML::Node RosDynamicConfigServer::onSet(const DynamicConfigServer::Key& key, const YAML::Node& new_values) {
  auto error = server_.set(key, new_values);
  auto info = server_.getInfo(key);
  if (!error.empty()) {
    info["error"] = error;
  }
  return info;
}

}  // namespace config
