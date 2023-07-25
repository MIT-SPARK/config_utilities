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
  // Copy operators.
  VirtualConfig() = default;
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
   * @brief Check whether the config is populated with a config.
   *
   * @returns True if the config is populated, false otherwise.
   */
  bool isSet() const { return config_.operator bool(); }
  operator bool() const { return isSet(); }

  /**
   * @brief Specify whether this config is optional. If it is optional, the config not being set is not an error.
   * Otherwise the config must be set to be considered valid. By default virtual configs are not optional.
   */
  void setOptional(bool optional = true) { optional_ = optional; }

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
    }
  }

  // If a config is contained, propagate the declaration to the contained object.
  if (config.config_) {
    config.config_->onDeclareConfig();
  }
}

}  // namespace config