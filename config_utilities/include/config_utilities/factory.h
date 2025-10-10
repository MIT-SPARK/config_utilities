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
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "config_utilities/internal/introspection.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"

namespace config {

namespace internal {

//! @brief Convenience typedef for underlying construct used by factory methods
template <typename BaseT, typename... Args>
using FactoryMethod = std::function<BaseT*(Args...)>;

//! @brief Convert parameter pack to vector of typenames
template <typename... Args>
std::vector<std::string> convertArguments() {
  return std::vector<std::string>{
      (typeName<Args>())...,
  };
}

/** @brief Helper function to read the type param from a node.
 * @param data YAML node to read type from
 * @param type Type value to filll
 * @param required Whether or not the type field is required
 * @param param_name Field in YAML node to read (empty string defaults to Settings().factory.type_param_name)
 */
bool getType(const YAML::Node& data, std::string& type, bool required = true, const std::string& param_name = "");

//! @brief Struct recording typenames for a module (i.e., the constructor signature). Can be used as a map key
struct ModuleInfo {
  template <typename BaseT, typename... Args>
  static ModuleInfo fromTypes(bool skip_first_arg = false, const std::string underlying_base = "") {
    return {skip_first_arg, typeName<BaseT>(), convertArguments<Args...>(), underlying_base};
  }

  //! Toggle whether the first argument is included (for printing only)
  bool skip_first_arg;
  //! Factory function return type
  const std::string base_type;
  //! Factory function arguments
  const std::vector<std::string> arguments;
  //! Underlying base type (for configs)
  const std::string underlying_base;

  //! @brief Get a human-readable signature for arguments
  std::string argumentString(const std::string& separator = ", ",
                             const std::string& wrapper = "",
                             const std::string& placeholder = "") const;
  //! @brief Get the full signature in a human-readable format
  std::string typeInfo() const;
  //! @brief Get the human-readable function signature
  std::string signature() const;
};

bool operator<(const ModuleInfo& lhs, const ModuleInfo& rhs);

//! @brief Struct recording correspondence between Object and Config for ObjectWithConfig
struct ConfigPair {
  template <typename BaseT, typename ConfigT>
  static ConfigPair fromTypes() {
    return {typeName<BaseT>(), typeName<ConfigT>()};
  }

  std::string base_type;
  std::string config_type;
};

bool operator<(const ConfigPair& lhs, const ConfigPair& rhs);

//! @brief Wrapper struct for any config type.
struct ConfigWrapper {
  explicit ConfigWrapper(const std::string& _type) : type(_type) {}
  virtual ~ConfigWrapper() = default;
  virtual std::unique_ptr<ConfigWrapper> clone() const = 0;
  virtual void onDeclareConfig() = 0;
  virtual std::unique_ptr<ConfigWrapper> createDefault() const = 0;
  std::string type;
};

//! @brief Virtual base class for factories to hide constructor type
struct FactoryMapBase {
  virtual ~FactoryMapBase() = default;
  virtual bool hasEntry(const std::string& type) = 0;
  virtual void removeEntry(const std::string& type) = 0;
  virtual bool empty() = 0;
};

//! @brief Struct to store the factory methods for the creation of modules.
template <typename Constructor>
struct FactoryMap : FactoryMapBase {
  //! @brief Add entries to the map with verbose warnings.
  bool addEntry(const std::string& type, const Constructor& method) {
    if (hasEntry(type)) {
      return false;
    }

    map.emplace(type, method);
    return true;
  }

  //! @brief Get factory for type (returns an invalid function if type not registered)
  Constructor getEntry(const std::string& type) {
    auto iter = map.find(type);
    return iter == map.end() ? Constructor() : iter->second;
  }

  bool hasEntry(const std::string& type) override { return map.find(type) != map.end(); }
  void removeEntry(const std::string& type) override { map.erase(type); }
  bool empty() override { return map.empty(); }

 private:
  std::map<std::string, Constructor> map;
};

//! @brief Registry for all factories and config type names.
class ModuleRegistry {
 public:
  using LockCallback = std::function<void(const ModuleInfo&, std::string, std::string)>;
  using CreateCallback = std::function<void(const ModuleInfo&, std::string, void*)>;

  template <typename BaseT, typename DerivedT, typename... Args>
  static bool addModule(const std::string& type,
                        FactoryMethod<BaseT, Args...> method,
                        bool skip_first_arg = false,
                        const std::string& actual_base = "") {
    const auto key = ModuleInfo::fromTypes<BaseT, Args...>(skip_first_arg, actual_base);
    using Constructor = FactoryMethod<BaseT, Args...>;
    if (locked()) {
      if (hasModule(key, type)) {
        // skip adding module that is registered in the main executable already
        // without spamming the user with print statements. Duplicate external modules should be caught by the external
        // registry
        return false;
      }

      const auto& callback = instance().lock_callback_;
      if (callback) {
        callback(key, type, typeName<DerivedT>());
      }
    }

    auto& modules = instance().modules;
    auto iter = modules.find(key);
    if (iter == modules.end()) {
      iter = modules.emplace(key, std::make_unique<FactoryMap<Constructor>>()).first;
    }

    auto derived = dynamic_cast<FactoryMap<Constructor>*>(iter->second.get());
    if (!derived) {
      Logger::logFatal("Invalid module map for type '" + type + "' and info " + key.typeInfo());
      return false;
    }

    if (!derived->addEntry(type, method)) {
      Logger::logError("Cannot register already existent type '" + type + "' for " + key.typeInfo() + ".");
      return false;
    }

    instance().type_registry[key][type] = typeName<DerivedT>();
    return true;
  }

  template <typename BaseT, typename... Args>
  static FactoryMethod<BaseT, Args...> getModule(const std::string& type,
                                                 const std::string& registration_info,
                                                 bool skip_first_arg = false,
                                                 const std::string& actual_base = "") {
    using Constructor = FactoryMethod<BaseT, Args...>;
    const auto& modules = instance().modules;
    const auto key = ModuleInfo::fromTypes<BaseT, Args...>(skip_first_arg, actual_base);
    const auto iter = modules.find(key);
    if (iter == modules.end()) {
      Logger::logError("Cannot create a module of type '" + type + "': No modules registered to the factory for " +
                       key.typeInfo() + ". Register modules using a static " + registration_info + " struct.\n" +
                       getAllRegistered());
      return {};
    }

    auto derived = dynamic_cast<FactoryMap<Constructor>*>(iter->second.get());
    if (!derived) {
      Logger::logFatal("Invalid module map for type '" + type + "' and info " + key.typeInfo());
      return {};
    }

    const auto factory = derived->getEntry(type);
    if (!factory) {
      // warn user if they specified a type but there was no corresponding factory
      if (!type.empty()) {
        Logger::logError("No module of type '" + type + "' registered to the factory for " + key.typeInfo() +
                         ". Registered are: '" + getRegistered(key) + "'.");
      }
      return factory;
    }

    const auto& create_callback = instance().create_callback_;
    if (!create_callback) {
      // just return factory if no external libraries are in play
      return factory;
    }

    // wrap factory call to register any allocations
    return [factory, key, type, create_callback](Args... args) -> BaseT* {
      auto pointer = factory(std::move(args)...);
      create_callback(key, type, pointer);
      return pointer;
    };
  }

  template <typename BaseT, typename... Args>
  static void removeModule(const std::string& type, bool skip_first_arg = false, const std::string& actual_base = "") {
    const auto key = ModuleInfo::fromTypes<BaseT, Args...>(skip_first_arg, actual_base);
    removeModule(key, type);
  }

  template <typename BaseT, typename ConfigT>
  static void registerConfig(const std::string& type) {
    const auto key = ConfigPair::fromTypes<BaseT, ConfigT>();
    auto& registry = instance().config_registry;
    auto iter = registry.find(key);
    if (iter == registry.end()) {
      registry.emplace(key, type);
      return;
    }

    // NOTE(lschmid): This is not forbidden behavior, but is not recommended so for now simply warn the user.
    if (iter != registry.end() && iter->second != type) {
      Logger::logInfo("Overwriting type name for config '" + typeName<ConfigT>() + "' for base module '" +
                      typeName<BaseT>() + "' from '" + iter->second + "' to '" + type +
                      "'. Defining different type identifiers for the same derived module is not recommended.");
      iter->second = type;
    }
  }

  template <typename BaseT, typename ConfigT>
  static void removeConfig() {
    const auto key = ConfigPair::fromTypes<BaseT, ConfigT>();
    auto& registry = instance().config_registry;
    registry.erase(key);
  }

  template <typename BaseT, typename ConfigT>
  static std::string getType() {
    auto& registry = instance().config_registry;
    auto iter = registry.find(ConfigPair::fromTypes<BaseT, ConfigT>());
    return iter == registry.end() ? "" : iter->second;
  }

  static std::vector<std::string> getRegisteredConfigTypes(const std::string& actual_base) {
    const auto key = ModuleInfo::fromTypes<ConfigWrapper>(false, actual_base);
    const auto& registry = instance().type_registry;
    const auto iter = registry.find(key);
    if (iter == registry.end()) {
      return {};
    }

    std::vector<std::string> result;
    for (const auto& [type, _] : iter->second) {
      result.push_back(type);
    }
    return result;
  }

  static bool hasModule(const ModuleInfo& key, const std::string& type);
  static void removeModule(const ModuleInfo& key, const std::string& type);
  static std::string getAllRegistered();
  static std::string getRegistered(const ModuleInfo& module);
  static void lock(LockCallback callback);
  static void unlock();
  static bool locked();
  static void setCreationCallback(CreateCallback callback);

 private:
  static std::unique_ptr<ModuleRegistry> s_instance_;
  static ModuleRegistry& instance();
  ModuleRegistry() = default;

  bool locked_;
  LockCallback lock_callback_;
  CreateCallback create_callback_;

  std::map<ModuleInfo, std::unique_ptr<FactoryMapBase>> modules;
  // Nested modules: base_type + args -> registered <type_name, type>.
  std::map<ModuleInfo, std::map<std::string, std::string>> type_registry;
  // Mapping between BaseT + ConfigT and underlying string type
  std::map<ConfigPair, std::string> config_registry;
};  // namespace internal

//! @brief Base class to make partial specialization work
template <typename T>
struct ModuleMapBase {};

template <typename ConfigT>
struct ConfigWrapperImpl : public ConfigWrapper {
  explicit ConfigWrapperImpl(const std::string& _type) : ConfigWrapper(_type) {}
  ConfigT config;
  std::unique_ptr<ConfigWrapper> clone() const override { return std::make_unique<ConfigWrapperImpl<ConfigT>>(*this); };
  void onDeclareConfig() override { ::config::declare_config(config); }
  std::unique_ptr<ConfigWrapper> createDefault() const override {
    return std::make_unique<ConfigWrapperImpl<ConfigT>>(type);
  };
};

// Definitions of the Factories.
// Factory to create configs.
template <class BaseT>
struct ConfigFactory {
  using Constructor = FactoryMethod<ConfigWrapper>;
  inline static constexpr auto registration_info =
      "config::RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>";

  // Add entries.
  template <class DerivedConfigT>
  static void addEntry(const std::string& type) {
    const Constructor method = [type]() -> ConfigWrapper* { return new ConfigWrapperImpl<DerivedConfigT>(type); };
    if (ModuleRegistry::addModule<ConfigWrapper, DerivedConfigT>(type, method, false, typeName<BaseT>())) {
      ModuleRegistry::registerConfig<BaseT, DerivedConfigT>(type);
    }
  }

  template <typename DerivedConfigT>
  static void removeEntry(const std::string& type) {
    ModuleRegistry::removeModule<ConfigWrapper>(type, false, typeName<BaseT>());
    ModuleRegistry::removeConfig<BaseT, DerivedConfigT>();
  }

  // Create the config.
  static std::unique_ptr<ConfigWrapper> create(const std::string& type) {
    const auto factory = ModuleRegistry::getModule<ConfigWrapper>(type, registration_info, false, typeName<BaseT>());
    if (!factory) {
      return nullptr;
    }

    return std::unique_ptr<ConfigWrapper>(factory());
  }
};

// Factory to create DerivedT objects without configs.
template <class BaseT, typename... Args>
struct ObjectFactory {
  using Constructor = FactoryMethod<BaseT, Args...>;
  inline static constexpr auto registration_info = "config::Registration<BaseT, DerivedT, ConstructorArguments...>";

  // Add entries.
  template <typename DerivedT>
  static void addEntry(const std::string& type) {
    const Constructor method = [](Args... args) -> BaseT* { return new DerivedT(std::move(args)...); };
    ModuleRegistry::addModule<BaseT, DerivedT, Args...>(type, method);
  }

  static void removeEntry(const std::string& type) { ModuleRegistry::removeModule<BaseT, Args...>(type); }

  static std::unique_ptr<BaseT> create(const std::string& type, Args... args) {
    const auto factory = ModuleRegistry::getModule<BaseT, Args...>(type, registration_info);
    if (!factory) {
      return nullptr;
    }

    return std::unique_ptr<BaseT>(factory(std::move(args)...));
  }
};

// Factory to create DerivedT objects that use configs.
template <class BaseT, typename... Args>
struct ObjectWithConfigFactory {
  using Constructor = FactoryMethod<BaseT, const YAML::Node&, Args...>;
  inline static constexpr auto registration_info =
      "config::RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>";

  // Add entries.
  template <typename DerivedT, typename DerivedConfigT>
  static void addEntry(const std::string& type) {
    const Constructor method = [type](const YAML::Node& data, Args... args) -> BaseT* {
      DerivedConfigT config;
      auto set_data = Visitor::setValues(config, data);
      if (Settings::instance().introspection.enabled()) {
        auto get_info = Visitor::getInfo(config);
        // Also log the type parameter used for creation.
        auto& set_field = set_data.field_infos.emplace_back();
        set_field.name = Settings::instance().factory.type_param_name;
        set_field.value = type;
        set_field.is_meta_field = true;
        auto& get_field = get_info.field_infos.emplace_back();
        get_field = set_field;
        set_field.was_parsed = true;
        Introspection::logSetValue(set_data, get_info);
      }
      return new DerivedT(config, std::move(args)...);
    };

    ModuleRegistry::addModule<BaseT, DerivedT, const YAML::Node&, Args...>(type, method, true);
  }

  static void removeEntry(const std::string& type) {
    ModuleRegistry::removeModule<BaseT, const YAML::Node&, Args...>(type, true);
  }

  static std::unique_ptr<BaseT> create(const YAML::Node& data, Args... args) {
    std::string type;
    if (!getType(data, type)) {
      if (Settings::instance().introspection.enabled()) {
        // Log that the type could not be determined.
        Introspection::logSingleEvent(
            Introspection::Event(Introspection::Event::Type::GetError, Introspection::By::config("Factory::create()")),
            Settings::instance().factory.type_param_name);
      }
      return nullptr;
    }

    const auto factory = ModuleRegistry::getModule<BaseT, const YAML::Node&, Args...>(type, registration_info, true);
    if (!factory) {
      return nullptr;
    }

    return std::unique_ptr<BaseT>(factory(data, std::move(args)...));
  }
};

}  // namespace internal

/**
 * @brief Allocate these structs statically to add entries to the factory for run-time creation of modules that use only
 * the constructor arguments based on string identifiers.
 *
 * @tparam BaseT Type of the base class to be registered. The base classes are queried for creation.
 * @tparam DerivedT Type of the derived class to register.
 * @tparam Args... The constructor arguments for the creation of objects. Notice that each unique set of constructor
 * arguments will result in a different base-entry in the factory and thus have to be registered separately.
 * @param type String identifier to look up and to create this derived type.
 */
template <class BaseT, class DerivedT, typename... ConstructorArguments>
struct Registration {
  explicit Registration(const std::string& type) {
    internal::ObjectFactory<BaseT, ConstructorArguments...>::template addEntry<DerivedT>(type);
  }
};

/**
 * @brief Allocate these structs statically to add entries to the factory for run-time creation of modules that use
 * configs. The specified config needs to be delcared a config by implementing 'void declare_config(DerivedConfigT&
 * config)'. Note that the config is expected to be the first constructor argument, followed by optional other
 * arguments.
 *
 * @tparam BaseT Type of the base class to be registered. The base classes are queried for creation.
 * @tparam DerivedT Type of the derived class to register.
 * @tparam DerivedConfigT Type of the config to be used for the creation of the derived class.
 * @tparam Args... The constructor arguments for the creation of objects. Notice that each unique set of constructor
 * arguments will result in a different base-entry in the factory and thus have to be registered separately.
 * @param type String identifier to look up and to create this derived type.
 */
template <class BaseT, class DerivedT, class DerivedConfigT, typename... ConstructorArguments>
struct RegistrationWithConfig {
  explicit RegistrationWithConfig(const std::string& type) {
    internal::ConfigFactory<BaseT>::template addEntry<DerivedConfigT>(type);
    internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::template addEntry<DerivedT, DerivedConfigT>(
        type);
  }
};

/**
 * @brief Create a derived type object based on a string identifier. All derived types need to be registered to the
 * factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct.
 *
 * @tparam BaseT Type of the base class to query for.
 * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
 * different base-entry in the factory.
 * @param type String identifier of the derived type to create.
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> create(const std::string& type, ConstructorArguments... args) {
  return internal::ObjectFactory<BaseT, ConstructorArguments...>::create(type, std::move(args)...);
}

}  // namespace config
