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
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "config_utilities/factory.h"
#include "config_utilities/traits.h"

namespace config {

/**
 * @brief The virtual config is a config struct that wraps an arbitrary config struct for later creation of a DerivedT
 * class.
 *
 * @tparam BaseT The base class of the object that should be created from the config.
 */
template <class BaseT>
class VirtualConfig {
 public:
  VirtualConfig() = default;
  ~VirtualConfig() = default;

  // Copy and assignment operators.
  VirtualConfig(const VirtualConfig& other) {
    if (other.config_) {
      config_ = other.config_->clone();
    }
    optional_ = other.optional_;
  }

  VirtualConfig(VirtualConfig&& other) {
    config_ = std::move(other.config_);
    optional_ = other.optional_;
  }

  VirtualConfig& operator=(const VirtualConfig& other) {
    if (other.config_) {
      config_ = other.config_->clone();
    }
    optional_ = other.optional_;
    return *this;
  }

  VirtualConfig& operator=(VirtualConfig&& other) {
    config_ = std::move(other.config_);
    optional_ = other.optional_;
    return *this;
  }

  /**
   * @brief Assign a config to this virtual config. This will check that the config being assigned is registered for a
   * module inheritin from the base class of this virtual config, and will use the registered type-string as type.
   * NOTE: If the same config is registered with different names for different constructor arguments, config assignments
   * may fail to retrieve the correct name.
   * @tparam ConfigT The type of the config to assign.
   * @param config The config to assign.
   * @returns True if the config was set successfully, false otherwise.
   */
  template <typename ConfigT>
  bool set(const ConfigT& config) {
    const std::string type = internal::ConfigTypeRegistry<BaseT, ConfigT>::getType();
    if (type.empty()) {
      // No type defined for the config.
      internal::Logger::logError("No module for config '" + internal::typeName<ConfigT>() +
                                 "' is registered to the factory for '" + internal::typeName<BaseT>() +
                                 "' to set virtual config.");
      return false;
    }

    // Assign the config.
    auto wrapper = std::make_unique<internal::ConfigWrapperImpl<ConfigT>>(type);
    wrapper->config = config;
    config_ = std::move(wrapper);
    return true;
  }

  template <typename ConfigT>
  explicit VirtualConfig(const ConfigT& config) {
    set(config);
  }

  template <typename ConfigT>
  VirtualConfig& operator=(const ConfigT& config) {
    set(config);
    return *this;
  }

  /**
   * @brief Check whether the config is populated with a config.
   *
   * @returns True if the config is populated, false otherwise.
   */
  bool isSet() const { return config_.operator bool(); }
  operator bool() const { return isSet(); }

  /**
   * @brief Specify whether this config is optional. If it is optional, the config not being set is not an error.
   * Otherwise the config must be set to be considered valid. By default virtual configs are not optional.
   * @param optional Turn optional on (true) or off (false).
   */
  void setOptional(bool optional = true) { optional_ = optional; }

  bool optional() const { return optional_; }

  /**
   * @brief Get the string-identifier-type of the config stored in the virtual config.
   */
  std::string getType() const { return config_ ? config_->type : "Uninitialized"; }

  /**
   * @brief Create the DerivedT object specified in the config.
   *
   * @tparam ConstructorArguments Type of ptional additional arguments to pass to the constructor of DerivedT.
   * @param args Optional additional arguments to pass to the constructor of DerivedT.
   * @return The created object of DerivedT that inherits from BaseT.
   */
  template <typename... ConstructorArguments>
  std::unique_ptr<BaseT> create(ConstructorArguments... args) const {
    if (!config_) {
      return nullptr;
    }
    // NOTE(lschmid): This is not the most beautiful but fairly general. Deserialize the config to YAML and use that to
    // create the object with the standard factory. We assume that every type that can be serialized into a config can
    // also be de-serialized so this should not result in any warnings, we print them anyways to be sure. The factory
    // should take proper care of any other verbose error management.
    const internal::MetaData data = internal::Visitor::getValues(*this);
    return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(data.data, args...);
  }

 private:
  template <typename T>
  friend void declare_config(VirtualConfig<T>&);
  friend struct internal::Visitor;

  bool optional_ = false;
  std::unique_ptr<internal::ConfigWrapper> config_;
};

namespace internal {

// Declare virtual config types.
template <typename T>
struct is_virtual_config<VirtualConfig<T>> : std::true_type {};

}  // namespace internal

// Declare the Virtual Config a config, so it can be handled like any other object.
template <typename BaseT>
void declare_config(VirtualConfig<BaseT>& config) {
  std::optional<YAML::Node> data =
      internal::Visitor::visitVirtualConfig(config.isSet(), config.optional_, config.getType());

  // If setting values create the wrapped config using the string identifier.
  if (data) {
    std::string type;
    const bool success = config.optional_ ? internal::getTypeImpl(*data, type, Settings().factory_type_param_name)
                                          : internal::getType(*data, type);
    if (success) {
      config.config_ = internal::ConfigFactory<BaseT>::create(type);
    } else if (!config.optional_) {
      std::stringstream ss;
      ss << "Could not get type for '" << internal::typeInfo<BaseT>() << "'";
      internal::Logger::logError(ss.str());
    }
  }

  // If a config is contained, propagate the declaration to the contained object.
  if (config.config_) {
    config.config_->onDeclareConfig();
  }
}

}  // namespace config
