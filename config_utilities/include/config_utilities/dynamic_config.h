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

#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <config_utilities/internal/visitor.h>
#include <config_utilities/traits.h>
#include <config_utilities/validation.h>

namespace config {

/**
 * @brief A server interface to manage all dynamic configs.
 */
struct DynamicConfigServer {
  using Key = std::string;

  struct Hooks {
    std::function<void(const Key&)> onRegister;
    std::function<void(const Key&)> onDeregister;
    std::function<void(const Key&, const YAML::Node&)> onUpdate;

    bool empty() const;
  };

  DynamicConfigServer() = default;
  virtual ~DynamicConfigServer();
  DynamicConfigServer(const DynamicConfigServer&) = delete;
  DynamicConfigServer(DynamicConfigServer&&) = default;
  DynamicConfigServer& operator=(const DynamicConfigServer&) = delete;
  DynamicConfigServer& operator=(DynamicConfigServer&&) = default;

  /**
   * @brief Check if a dynamic config with the given key exists.
   * @param key The unique key of the dynamic config.
   * @return True if the dynamic config exists, false otherwise.
   */
  bool hasConfig(const Key& key) const;

  /**
   * @brief Get the keys of all registered dynamic configs.
   */
  std::vector<Key> registeredConfigs() const;

  /**
   * @brief Get the values of a dynamic config.
   * @param key The unique key of the dynamic config.
   */
  YAML::Node getValues(const Key& key) const;

  /**
   * @brief Set the values of a dynamic config.
   * @param key The unique key of the dynamic config.
   * @param values The new values to set.
   */
  void setValues(const Key& key, const YAML::Node& values) const;

  /**
   * @brief Set the hooks for the dynamic config server.
   */
  void setHooks(const Hooks& hooks);

  /**
   * @brief Get the info of a dynamic config.
   * @param key The unique key of the dynamic config.
   */
  YAML::Node getInfo(const Key& key) const;

 private:
  size_t hooks_id_ = 0;
};

namespace internal {

/**
 * @brief Name-based global registry for dynamic configurations.
 */
struct DynamicConfigRegistry {
  using Key = DynamicConfigServer::Key;

  /**
   * @brief Server-side interface to dynamic configs.
   */
  struct ConfigInterface {
    std::function<YAML::Node()> getValues;
    std::function<void(const YAML::Node&)> setValues;
    std::function<YAML::Node()> getInfo;
  };

  // Singleton access.
  static DynamicConfigRegistry& instance() {
    static DynamicConfigRegistry instance;
    return instance;
  }

  /**
   * @brief Check if a dynamic config with the given key is registered.
   */
  bool hasKey(const Key& key) const;

  /**
   * @brief Get the interface to a dynamic config with the given key.
   * @param key The unique key of the dynamic config.
   * @return The interface to the dynamic config, if it exists.
   */
  std::optional<ConfigInterface> getConfig(const Key& key) const;

  /**
   * @brief Get all keys of the registered dynamic configs.
   */
  std::vector<Key> keys() const;

  // Dynamic config registration and de-registration.
  /**
   * @brief Register a dynamic config with the given key.
   * @param key The unique key of the dynamic config.
   * @param interface The interface to the dynamic config.
   * @return True if the registration was successful, false otherwise.
   */
  bool registerConfig(const Key& key, const ConfigInterface& interface);

  /**
   * @brief De-register a dynamic config with the given key.
   * @param key The unique key of the dynamic config.
   */
  void deregisterConfig(const Key& key);

  /**
   * @brief Register hooks for a dynamic config server.
   * @param hooks The hooks to register.
   * @param hooks_id The id of the server adding the hooks.
   * @return The new_id of the server registered hooks.
   */
  size_t registerHooks(const DynamicConfigServer::Hooks& hooks, size_t hooks_id);

  /**
   * @brief Deregister hooks for a dynamic config server.
   */
  void deregisterHooks(size_t hooks_id);

  /**
   * @brief Notify all hooks that a config was updated.
   */
  void configUpdated(const Key& key, const YAML::Node& new_values);

 private:
  DynamicConfigRegistry() = default;

  std::unordered_map<Key, ConfigInterface> configs_;
  std::unordered_map<size_t, DynamicConfigServer::Hooks> hooks_;
  size_t current_hooks_id_ = 0;
};

}  // namespace internal

/**
 * @brief A wrapper class for for configs that can be dynamically changed.
 *
 * @tparam ConfigT The contained configuration type
 */
template <typename ConfigT>
struct DynamicConfig {
  /**
   * @brief Construct a new Dynamic Config, wrapping a config_uilities config.
   * @param name Unique name of the dynamic config. This identifier is used to access the config on the client side
   * @param config The config to wrap.
   */
  explicit DynamicConfig(const std::string& name, const ConfigT& config = {})
      : name_(name), config_(config::checkValid(config)) {
    static_assert(isConfig<ConfigT>(),
                  "ConfigT must be declared to be a config. Implement 'void declare_config(ConfigT&)'.");
    is_registered_ = internal::DynamicConfigRegistry::instance().registerConfig(
        name_,
        {std::bind(&DynamicConfig::getValues, this),
         std::bind(&DynamicConfig::setValues, this, std::placeholders::_1),
         std::bind(&DynamicConfig::getInfo, this)});
  }

  ~DynamicConfig() {
    if (is_registered_) {
      internal::DynamicConfigRegistry::instance().deregisterConfig(name_);
    }
  }

  DynamicConfig(const DynamicConfig&) = delete;
  DynamicConfig(DynamicConfig&&) = default;
  DynamicConfig& operator=(const DynamicConfig&) = delete;
  DynamicConfig& operator=(DynamicConfig&&) = default;

  /**
   * @brief Get the underlying dynamic config.
   * @note This returns a copy of the config, so changes to the returned config will not affect the dynamic config.
   */
  ConfigT get() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
  }

  /**
   * @brief Set the underlying dynamic config.
   */
  void set(const ConfigT& config) {
    if (!config::isValid(config)) {
      return;
    }
    if (!is_registered_) {
      config_ = config;
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const auto old_yaml = internal::Visitor::getValues(config_).data;
    const auto new_yaml = internal::Visitor::getValues(config).data;
    if (internal::isEqual(old_yaml, new_yaml)) {
      return;
    }
    config_ = config;
    internal::DynamicConfigRegistry::instance().configUpdated(name_, new_yaml);
  }

 private:
  const std::string name_;
  ConfigT config_;
  mutable std::mutex mutex_;
  bool is_registered_;

  void setValues(const YAML::Node& values) {
    std::lock_guard<std::mutex> lock(mutex_);
    // TODO(lschmid): We should check if the values are valid before setting them. Ideally field by field...
    internal::Visitor::setValues(config_, values);
  }

  YAML::Node getValues() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return internal::Visitor::getValues(config_).data;
  }

  YAML::Node getInfo() const {
    std::lock_guard<std::mutex> lock(mutex_);
    // TODO(lschmid): Add a visitor function to get the info of a config.
    return {};
  }
};

}  // namespace config
