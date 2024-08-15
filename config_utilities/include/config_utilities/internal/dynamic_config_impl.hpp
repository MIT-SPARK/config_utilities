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

#include <utility>

namespace config {

template <typename ConfigT>
DynamicConfig<ConfigT>::DynamicConfig(const std::string& name,
                                      const ConfigT& config,
                                      DynamicConfig<ConfigT>::Callback callback)
    : name_(name),
      config_(config::checkValid(config)),
      callback_(callback),
      is_registered_(internal::DynamicConfigRegistry::instance().registerConfig(name_, getInterface())) {
  static_assert(isConfig<ConfigT>(),
                "ConfigT must be declared to be a config. Implement 'void declare_config(ConfigT&)'.");
  static_assert(std::is_copy_constructible<ConfigT>::value, "ConfigT must be copy constructible.");
}

template <typename ConfigT>
DynamicConfig<ConfigT>::~DynamicConfig() {
  if (is_registered_) {
    internal::DynamicConfigRegistry::instance().deregisterConfig(name_);
  }
}

template <typename ConfigT>
DynamicConfig<ConfigT>::DynamicConfig(DynamicConfig&& other)
    : name_(other.name_), is_registered_(other.is_registered_) {
  moveMembers(std::move(other));
}

template <typename ConfigT>
DynamicConfig<ConfigT>& DynamicConfig<ConfigT>::operator=(DynamicConfig&& other) {
  if (this != &other) {
    moveMembers(std::move(other));
  }
  return *this;
}

template <typename ConfigT>
ConfigT DynamicConfig<ConfigT>::get() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

template <typename ConfigT>
bool DynamicConfig<ConfigT>::set(const ConfigT& config) {
  if (!config::isValid(config, true)) {
    return false;
  }
  if (!is_registered_) {
    config_ = config;
    // NOTE(lschmid): This returns true even if the config is the same as the old one. Might want to move this further
    // down in the future, although I don't think it matters since the user has control over the config here.
    return true;
  }

  const auto new_yaml = internal::Visitor::getValues(config).data;
  {  // critical section
    std::lock_guard<std::mutex> lock(mutex_);
    const auto old_yaml = internal::Visitor::getValues(config_).data;
    if (internal::isEqual(old_yaml, new_yaml)) {
      return false;
    }
    config_ = config;
  }  // end critical section

  internal::DynamicConfigRegistry::instance().configUpdated(name_, new_yaml);
  return true;
}

template <typename ConfigT>
void DynamicConfig<ConfigT>::setCallback(const Callback& callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = callback;
}

template <typename ConfigT>
bool DynamicConfig<ConfigT>::setValues(const YAML::Node& values) {
  YAML::Node new_yaml;
  {  // start critical section
    std::lock_guard<std::mutex> lock(mutex_);
    ConfigT new_config = config_;
    internal::Visitor::setValues(new_config, values);
    if (!config::isValid(new_config, true)) {
      return false;
    }

    // NOTE(lschmid): This is a bit cumbersome, but configs don't have to implement operator==, so we compare
    // their YAML representation. Can consider making this optional in the future in the global settings?
    const auto old_yaml = internal::Visitor::getValues(config_).data;
    new_yaml = internal::Visitor::getValues(new_config).data;
    if (internal::isEqual(old_yaml, new_yaml)) {
      return false;
    }
    config_ = new_config;
  }  // end critical section

  if (callback_) {
    callback_();
  }

  // Also notify other clients that the config has been updated.
  internal::DynamicConfigRegistry::instance().configUpdated(name_, new_yaml);
  return true;
}

template <typename ConfigT>
YAML::Node DynamicConfig<ConfigT>::getValues() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return internal::Visitor::getValues(config_).data;
}

template <typename ConfigT>
YAML::Node DynamicConfig<ConfigT>::getInfo() const {
  std::lock_guard<std::mutex> lock(mutex_);
  // TODO(lschmid): Add a visitor function to get the info of a config.
  return {};
}

template <typename ConfigT>
internal::DynamicConfigRegistry::ConfigInterface DynamicConfig<ConfigT>::getInterface() {
  internal::DynamicConfigRegistry::ConfigInterface interface;
  interface.getValues = [this]() { return getValues(); };
  interface.setValues = [this](const YAML::Node& values) { return setValues(values); };
  interface.getInfo = [this]() { return getInfo(); };
  return interface;
}

template <typename ConfigT>
void DynamicConfig<ConfigT>::moveMembers(DynamicConfig&& other) {
  config_ = std::move(other.config_);
  callback_ = std::move(other.callback_);
  const_cast<bool&>(is_registered_) = other.is_registered_;

  if (is_registered_) {
    if (name_ != other.name_) {
      internal::DynamicConfigRegistry::instance().deregisterConfig(name_);
    }
    internal::DynamicConfigRegistry::instance().overrideRegistration(other.name_, getInterface());
  }
  const_cast<std::string&>(name_) = std::move(other.name_);
}

}  // namespace config
