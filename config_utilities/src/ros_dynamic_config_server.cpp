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
#include "config_utilities/ros_dynamic_config_server.h"

namespace config {

RosDynamicConfigServer::ConfigReceiver::ConfigReceiver(const DynamicConfigServer::Key& key,
                                                       RosDynamicConfigServer* server,
                                                       rclcpp::Node& node)
    : key(key), server(server) {
  // sub = nh.subscribe(key + "/set", 1, &ConfigReceiver::callback, this);
}

// void RosDynamicConfigServer::ConfigReceiver::callback(const std_msgs::String& msg) {
//   const auto values = YAML::Load(msg.data);
//   server->onSet(key, values);
// }

RosDynamicConfigServer::RosDynamicConfigServer(rclcpp::Node* node) : node_(node) {
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  keys_pub_ = node_->create_publisher<std_msgs::msg::String>("dynamic_config_keys", qos);

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

void RosDynamicConfigServer::onRegister(const DynamicConfigServer::Key& key) {
  // value_publishers_[key] = nh_.advertise<std_msgs::String>(key + "/get", 1, true);
  // info_publishers_[key] = nh_.advertise<std_msgs::String>(key + "/info", 1, true);
  // subscribers_[key] = std::make_unique<ConfigReceiver>(key, this, nh_);

  // Update the list of keys.
  publishKeys();

  // Latch the current state of the config.
  onUpdate(key, server_.get(key));
}

void RosDynamicConfigServer::onDeregister(const DynamicConfigServer::Key& key) {
  // value_publishers_.erase(key);
  // info_publishers_.erase(key);
  // subscribers_.erase(key);

  // Update the list of keys.
  publishKeys();
}

void RosDynamicConfigServer::onUpdate(const DynamicConfigServer::Key& key, const YAML::Node& values) {
  // const auto it = value_publishers_.find(key);
  // if (it == value_publishers_.end()) {
  //   // Shouldn't happen but better to fail gracefully if people extend this.
  //   internal::Logger::logWarning("Tried to publish to dynamic config '" + key + "' without existing publisher.");
  //   return;
  // }

  // std_msgs::String msg;
  // msg.data = YAML::Dump(values);
  // it->second.publish(msg);

  // // For now also always publish the info. Can consider being smarter about this if this ever is a limitation.
  // const auto info_it = info_publishers_.find(key);
  // if (info_it == info_publishers_.end()) {
  //   return;
  // }
  // const auto info = server_.getInfo(key);
  // msg.data = YAML::Dump(info);
  // info_it->second.publish(msg);
}

void RosDynamicConfigServer::onSet(const DynamicConfigServer::Key& key, const YAML::Node& new_values) {
  server_.set(key, new_values);
}

void RosDynamicConfigServer::publishKeys() {
  // Publish the all of keys as yaml list.
  std_msgs::msg::String msg;

  msg.data = "[";
  for (const auto& config : configs_) {
    msg.data += config.key + ", ";
  }
  if (configs_.size() > 0) {
    msg.data = msg.data.substr(0, msg.data.size() - 2);
  }
  msg.data += "]";
  keys_pub_->publish(msg);
}

}  // namespace config
