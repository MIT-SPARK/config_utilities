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

  /**
   * @brief Hooks for the dynamic config server. These functions are called whenever a dynamic config is registered,
   * deregistered, or updated.
   */
  struct Hooks {
    std::function<void(const Key&)> onRegister;
    std::function<void(const Key&)> onDeregister;
    std::function<void(const Key&, const YAML::Node&)> onUpdate;

    bool empty() const;
  };

  DynamicConfigServer() = default;
  explicit DynamicConfigServer(const Hooks& hooks);
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
  YAML::Node get(const Key& key) const;

  /**
   * @brief Get the values and type info of a dynamic config.
   * @param key The unique key of the dynamic config.
   */
  YAML::Node getInfo(const Key& key) const;

  /**
   * @brief Set the values of a dynamic config. If the requested values are invalid, no modifications are made.
   * @param key The unique key of the dynamic config.
   * @param values The new values to set.
   * @return Error messages if present. If the config was set successfully, the string is empty.
   */
  std::string set(const Key& key, const YAML::Node& values) const;

  /**
   * @brief Set the hooks for the dynamic config server. Setting empty hooks will deregister the current hooks.
   */
  void setHooks(const Hooks& hooks);

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
    std::function<YAML::Node()> get;
    std::function<YAML::Node()> getInfo;
    std::function<std::string(const YAML::Node&)> set;
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
   * @brief Override an existing registration with a new interface when moving a dynamic config.
   * @param key The unique key of the dynamic config.
   * @param interface The new interface to the dynamic config.
   */
  void overrideRegistration(const Key& key, const ConfigInterface& interface);

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
 * @brief A wrapper class for for configs that can be dynamically changed. If the dynamic config is const, it will be
 * read-only of the underlying config, which can still be changed from external sources.
 *
 * @tparam ConfigT The contained configuration type.
 */
template <typename ConfigT>
struct DynamicConfig {
  using Callback = std::function<void()>;

  /**
   * @brief Construct a new Dynamic Config, wrapping a config_uilities config.
   * @param name Unique name of the dynamic config. This identifier is used to access the config on the client side.
   * @param config The config to wrap.
   * @param callback A callback function that is called whenever the config is updated. This is only called if the
   * values of the config change, not on every set request.
   */
  explicit DynamicConfig(const std::string& name, const ConfigT& config = {}, Callback callback = {});

  virtual ~DynamicConfig();

  DynamicConfig(const DynamicConfig&) = delete;
  DynamicConfig& operator=(const DynamicConfig&) = delete;
  DynamicConfig(DynamicConfig&&);
  DynamicConfig& operator=(DynamicConfig&&);

  /**
   * @brief Get the underlying dynamic config.
   * @note This returns a copy of the config, so changes to the returned config will not affect the dynamic config.
   */
  ConfigT get() const;

  /**
   * @brief Set the underlying dynamic config.
   * @param config The new config to set. If the config is invalid, no modifications are made.
   * @return True if the config was updated, false otherwise.
   */
  bool set(const ConfigT& config);

  /**
   * @brief Set the callback function that is called whenever the config is updated.
   * @param callback The callback function to be called.
   */
  void setCallback(const Callback& callback);

 private:
  const std::string name_;
  mutable ConfigT config_;
  mutable std::mutex mutex_;
  Callback callback_;
  const bool is_registered_;

  std::string setValues(const YAML::Node& values) const;
  YAML::Node getValues() const;
  YAML::Node getInfo() const;
  internal::DynamicConfigRegistry::ConfigInterface getInterface() const;
  void moveMembers(DynamicConfig&& other);
};

}  // namespace config

#include <config_utilities/internal/dynamic_config_impl.hpp>
