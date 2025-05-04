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
#include <std_msgs/msg/string.hpp>

#include "config_utilities/dynamic_config.h"

namespace config {

/**
 * @brief Dynamic config server that allows to set and get configs via ROS topics.
 */
class RosDynamicConfigServer {
 public:
  explicit RosDynamicConfigServer(rclcpp::Node* node);

 private:
  // Struct to that manages the exposure of each config
  struct ConfigReceiver {
    ConfigReceiver(const DynamicConfigServer::Key& key, RosDynamicConfigServer* server, rclcpp::Node& node);
    const DynamicConfigServer::Key key;
    const RosDynamicConfigServer* const server;
    //   void callback(const std_msgs::String& msg);
  };

  // TODO(lschmid): Figure out if we can use smart pointers here. This should allow nice wrapping in the node.
  rclcpp::Node* node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keys_pub_;
  std::vector<ConfigReceiver> configs_;
  DynamicConfigServer server_;

  void onRegister(const DynamicConfigServer::Key& key);
  void onDeregister(const DynamicConfigServer::Key& key);
  void onUpdate(const DynamicConfigServer::Key& key, const YAML::Node& new_values);
  void onSet(const DynamicConfigServer::Key& key, const YAML::Node& new_values);
  void publishKeys();
};

}  // namespace config
