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
 * @tparam OptionalByDefault Whether or not the virtual config is optional when constructed (useful for maps and vectors
 * of configs)
 */
template <class BaseT, bool OptionalByDefault = false>
class VirtualConfig {
 public:
  VirtualConfig() = default;
  ~VirtualConfig() = default;

  // Copy and assignment operators.
  VirtualConfig(const VirtualConfig& other) {
    if (other.config_) {
      config_ = other.config_->clone();
    } else {
      config_.reset();
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
    } else {
      config_.reset();
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
   * @brief Operators == and != checks for equality of the carried configs, i.e. operator== returns true if both are
   * unset or if both are set and the types and values are equal.
   */
  // TODO(lschmid): Think about enabling operator== for types where operator== is defined. Easy to do if we want to bump
  // to C++20 at some point.
  bool operator==(const VirtualConfig& other) const {
    if (!config_ && !other.config_) {
      return true;
    }
    if (!config_ || !other.config_) {
      return false;
    }
    if (config_->type != other.config_->type) {
      return false;
    }
    // Compare the YAML representation of the configs as a work around for now.
    const auto this_meta = internal::Visitor::getValues(*this);
    const auto other_meta = internal::Visitor::getValues(other);
    return internal::yamlToString(this_meta.data) == internal::yamlToString(other_meta.data);
  }
  bool operator!=(const VirtualConfig& other) const { return !(*this == other); }

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
    const std::string type = internal::ModuleRegistry::getType<BaseT, ConfigT>();
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
  std::string getType() const { return config_ ? config_->type : internal::kUninitializedVirtualConfigType; }

  /**
   * @brief Get the underlying config that this holds, if set
   * @tparam ConfigT Derived type of underlying config
   * @returns Pointer to the underlying config if valid and the same type, nullptr otherwise
   */
  template <typename ConfigT>
  const ConfigT* getUnderlying() const {
    if (!config_) {
      return nullptr;
    }

    auto derived_wrapper = dynamic_cast<const internal::ConfigWrapperImpl<ConfigT>*>(config_.get());
    if (!derived_wrapper) {
      return nullptr;
    }

    return &derived_wrapper->config;
  }

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
    return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(data.data, std::move(args)...);
  }

 private:
  template <typename T, bool Opt>
  friend void declare_config(VirtualConfig<T, Opt>&);
  friend struct internal::Visitor;

  bool optional_ = OptionalByDefault;
  std::unique_ptr<internal::ConfigWrapper> config_;
};

namespace internal {

// Declare virtual config types.
template <typename T, bool Opt>
struct is_virtual_config<VirtualConfig<T, Opt>> : std::true_type {};

}  // namespace internal

// Declare the Virtual Config a config, so it can be handled like any other object.
template <typename BaseT, bool Opt>
void declare_config(VirtualConfig<BaseT, Opt>& config) {
  const auto type = internal::Visitor::visitVirtualConfig(config.isSet(),
                                                          config.optional_,
                                                          config.getType(),
                                                          internal::typeName<BaseT>(),
                                                          internal::ModuleInfo::fromTypes<BaseT>().typeInfo());

  // If a type is returned the config should be reset or created.
  if (type) {
    if (*type == internal::kUninitializedVirtualConfigType) {
      // Reserved token to delete the virtual config in dynamic configs.
      config.config_.reset();
    } else {
      config.config_ = internal::ConfigFactory<BaseT>::create(*type);
    }
  }

  // If a config is contained, propagate the declaration to the contained object.
  if (config.config_) {
    config.config_->onDeclareConfig();
  }
}

}  // namespace config
