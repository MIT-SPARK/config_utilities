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

#include "config_utilities/dynamic_config.h"

#include "config_utilities/internal/logger.h"

namespace config {

bool DynamicConfigServer::Hooks::empty() const { return !onRegister && !onDeregister && !onUpdate; }

DynamicConfigServer::DynamicConfigServer(const Hooks& hooks) { setHooks(hooks); }

bool DynamicConfigServer::hasConfig(const Key& key) const {
  return internal::DynamicConfigRegistry::instance().hasKey(key);
}

std::vector<DynamicConfigServer::Key> DynamicConfigServer::registeredConfigs() const {
  return internal::DynamicConfigRegistry::instance().keys();
}

YAML::Node DynamicConfigServer::get(const Key& key) const {
  const auto config = internal::DynamicConfigRegistry::instance().getConfig(key);
  if (!config) {
    return {};
  }
  return config->get();
}

YAML::Node DynamicConfigServer::getInfo(const Key& key) const {
  const auto config = internal::DynamicConfigRegistry::instance().getConfig(key);
  if (!config) {
    return {};
  }
  return config->getInfo();
}

std::string DynamicConfigServer::set(const Key& key, const YAML::Node& values) const {
  const auto config = internal::DynamicConfigRegistry::instance().getConfig(key);
  if (!config) {
    return "No config registered with key '" + key + "'.";
  }
  return config->set(values);
}

void DynamicConfigServer::setHooks(const Hooks& hooks) {
  if (hooks.empty()) {
    internal::DynamicConfigRegistry::instance().deregisterHooks(hooks_id_);
    return;
  }
  hooks_id_ = internal::DynamicConfigRegistry::instance().registerHooks(hooks, hooks_id_);
}

DynamicConfigServer::~DynamicConfigServer() { internal::DynamicConfigRegistry::instance().deregisterHooks(hooks_id_); }

namespace internal {

bool DynamicConfigRegistry::hasKey(const Key& key) const { return configs_.count(key); }

std::optional<DynamicConfigRegistry::ConfigInterface> DynamicConfigRegistry::getConfig(const Key& key) const {
  const auto it = configs_.find(key);
  if (it == configs_.end()) {
    return std::nullopt;
  }
  return it->second;
}

std::vector<DynamicConfigRegistry::Key> DynamicConfigRegistry::keys() const {
  std::vector<Key> keys;
  keys.reserve(configs_.size());
  for (const auto& [key, _] : configs_) {
    keys.push_back(key);
  }
  return keys;
}

bool DynamicConfigRegistry::registerConfig(const Key& key, const ConfigInterface& interface) {
  if (configs_.count(key)) {
    Logger::logWarning("Cannot register dynamic config: key '" + key + "' already exists.");
    return false;
  }
  configs_[key] = interface;
  for (const auto& [hooks_id, hooks] : hooks_) {
    if (hooks.onRegister) {
      hooks.onRegister(key);
    }
  }
  return true;
}

void DynamicConfigRegistry::deregisterConfig(const Key& key) {
  auto it = configs_.find(key);
  if (it == configs_.end()) {
    return;
  }
  configs_.erase(key);
  for (const auto& [hooks_id, hooks] : hooks_) {
    if (hooks.onDeregister) {
      hooks.onDeregister(key);
    }
  }
}

void DynamicConfigRegistry::overrideRegistration(const Key& key, const ConfigInterface& interface) {
  configs_[key] = interface;
}

size_t DynamicConfigRegistry::registerHooks(const DynamicConfigServer::Hooks& hooks, size_t hooks_id) {
  if (hooks.empty()) {
    return hooks_id;
  }

  if (hooks_id == 0) {
    ++current_hooks_id_;
  }

  hooks_[hooks_id] = hooks;
  return hooks_id;
}

void DynamicConfigRegistry::deregisterHooks(size_t hooks_id) { hooks_.erase(hooks_id); }

void DynamicConfigRegistry::configUpdated(const Key& key, const YAML::Node& new_values) {
  for (const auto& [hooks_id, hooks] : hooks_) {
    if (hooks.onUpdate) {
      hooks.onUpdate(key, new_values);
    }
  }
}

}  // namespace internal

}  // namespace config
