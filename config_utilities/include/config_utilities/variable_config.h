#pragma once

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "config_utilities/factory.h"
#include "config_utilities/traits.h"

namespace config {

template <class BaseT>
class VariableConfig;

namespace internal {

template <typename ConfigT>
MetaData getDefaultValues(const VariableConfig<ConfigT>& config);

}  // namespace internal

/**
 * @brief The variable config is a config struct that wraps an arbitrary config struct for later creation of a DerivedT
 * class.
 */
template <class BaseT>
class VariableConfig {
 public:
  // Copy operators.
  VariableConfig() = default;
  VariableConfig(const VariableConfig& other) { config_ = other.config_->clone(); }
  VariableConfig(VariableConfig&& other) { config_ = std::move(other.config_); }
  VariableConfig& operator=(const VariableConfig& other) {
    config_ = other.config_->clone();
    return *this;
  }
  VariableConfig& operator=(VariableConfig&& other) {
    config_ = std::move(other.config_);
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
   * Otherwise the config must be set to be considered valid.
   */
  void setOptional(bool optional) { optional_ = optional; }

  /**
   * @brief Get the string-identifier-type of the config stored in the variable config.
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
    // NOTE(lschmid): This is not the most beautiful but fairly general. Deserialize the config to YAML and use that to
    // create the object with the standard factory. We assume that every type that can be serialized into a config can
    // also be de-serialized so this should not result in any warnings, we print them anyways to be sure. The factory
    // should take proper care of any other verbose error management.
    const internal::MetaData data = internal::Visitor::getValues(*this);
    return internal::Factory::createWithConfig<BaseT, ConstructorArguments...>(data.data, args...);
  }

 private:
  template <typename T>
  friend void declare_config(VariableConfig<T>&);
  template <typename T>
  friend internal::MetaData internal::getDefaultValues(const VariableConfig<T>&);

  bool optional_ = false;
  std::unique_ptr<internal::ConfigWrapper> config_;
};

namespace internal {

// Declare variable config types.
template <typename T>
struct is_variable_config<VariableConfig<T>> : std::true_type {};

// Specialization for default values.
template <typename ConfigT>
MetaData getDefaultValues(const VariableConfig<ConfigT>& config) {
  if (!config.config_) {
    return MetaData();
  }

  return config.config_->getDefaultValues();
}

}  // namespace internal

// Declare the Variable Config a config, so it can be handled like any other object.
template <typename BaseT>
void declare_config(VariableConfig<BaseT>& config) {
  std::optional<YAML::Node> data =
      internal::Visitor::visitVariableConfig(config.isSet(), config.optional_, config.getType());
  if (data) {
    // Create the wrapped config for the first time.
    config.config_ = internal::Factory::createConfig<BaseT>(*data);
  }
  if (config.config_) {
    config.config_->onDeclareConfig();
  }
}

}  // namespace config
