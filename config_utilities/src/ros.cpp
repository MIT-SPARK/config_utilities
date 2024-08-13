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
#include "config_utilities/parsing/ros.h"

namespace config {

RosDynamicConfigServer::ConfigReceiver::ConfigReceiver(const DynamicConfigServer::Key& key,
                                                       RosDynamicConfigServer* server,
                                                       ros::NodeHandle& nh)
    : key(key), server(server) {
  sub = nh.subscribe(key + "/set", 1, &ConfigReceiver::callback, this);
}

void RosDynamicConfigServer::ConfigReceiver::callback(const std_msgs::String& msg) {
  const auto values = YAML::Load(msg.data);
  server->onUpdate(key, values);
}

RosDynamicConfigServer::RosDynamicConfigServer(const ros::NodeHandle& nh) : nh_(nh) {
  reg_pub_ = nh_.advertise<std_msgs::String>("registered", 1);
  dereg_pub_ = nh_.advertise<std_msgs::String>("deregistered", 1);

  DynamicConfigServer::Hooks hooks;
  hooks.onRegister = [this](const DynamicConfigServer::Key& key) { onRegister(key); };
  hooks.onDeregister = [this](const DynamicConfigServer::Key& key) { onDeregister(key); };
  hooks.onUpdate = [this](const DynamicConfigServer::Key& key, const YAML::Node& values) { onUpdate(key, values); };
  server_.setHooks(hooks);
}

void RosDynamicConfigServer::onRegister(const DynamicConfigServer::Key& key) {
  publishers_[key] = nh_.advertise<std_msgs::String>(key + "/get", 1, true);
  subscribers_[key] = std::make_unique<ConfigReceiver>(key, this, nh_);
  std_msgs::String msg;
  msg.data = key;
  reg_pub_.publish(msg);
}

void RosDynamicConfigServer::onDeregister(const DynamicConfigServer::Key& key) {
  publishers_.erase(key);
  subscribers_.erase(key);
  std_msgs::String msg;
  msg.data = key;
  dereg_pub_.publish(msg);
}

void RosDynamicConfigServer::onUpdate(const DynamicConfigServer::Key& key, const YAML::Node& values) {
  server_.setValues(key, values);
}

}  // namespace config
