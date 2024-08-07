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

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config {

namespace internal {

template <typename BaseT, typename... Args>
using FactoryMethod = std::function<BaseT*(Args...)>;

template <typename... Args>
std::string joinArguments(const std::string& separator = ", ") {
  std::stringstream ss;
  ((ss << typeName<Args>() << separator), ...);
  if (ss.str().empty()) {
    return "";
  }

  return ss.str().substr(0, ss.str().size() - separator.size());
}

// Helper function to get human readable type infos.
template <class BaseT, typename... Args>
std::string typeInfo() {
  return "BaseT='" + typeName<BaseT>() + "' and ConstructorArguments={'" + joinArguments<Args...>("', ") + "'}";
}

struct ModuleKey {
  template <typename BaseT, typename... Args>
  static ModuleKey fromTypes() {
    return {typeName<BaseT>(), joinArguments<Args...>(), typeInfo<BaseT, Args...>()};
  }

  const std::string base_type;
  const std::string arguments;
  const std::string type_info;
};

bool operator<(const ModuleKey& lhs, const ModuleKey& rhs);

// Wrapper struct for any config type.
struct ConfigWrapper {
  explicit ConfigWrapper(const std::string& _type) : type(_type) {}
  virtual ~ConfigWrapper() = default;
  virtual std::unique_ptr<ConfigWrapper> clone() const = 0;
  virtual void onDeclareConfig() = 0;
  virtual std::unique_ptr<ConfigWrapper> createDefault() const = 0;
  std::string type;
};

struct ModuleMapBase {
  virtual ~ModuleMapBase() = default;
};

// Struct to store the factory methods for the creation of modules.
template <typename Factory>
struct ModuleMap : ModuleMapBase {
  using FactoryMethodMap = std::unordered_map<std::string, Factory>;

  ModuleMap() = default;

  // Add entries to the map with verbose warnings.
  bool addEntry(const std::string& type, const Factory& method) {
    const auto has_type = map.find(type) != map.end();
    if (has_type) {
      return false;
    }

    map.emplace(type, method);
    return true;
  }

  // Check if a requested type is valid with verbose warnings.
  Factory getEntry(const std::string& type) {
    auto iter = map.find(type);
    return iter == map.end() ? Factory() : iter->second;
  }

 private:
  // The map.
  FactoryMethodMap map;
};

class ModuleRegistry {
 public:
  template <typename BaseT, typename DerivedT, typename... Args>
  static void addModule(const std::string& type, FactoryMethod<BaseT, Args...> method) {
    using Factory = FactoryMethod<BaseT, Args...>;
    const auto key = ModuleKey::fromTypes<BaseT, Args...>();
    if (locked()) {
      Logger::logError("Adding type '" + type + "' for " + key.type_info + " when locked.");
    }

    auto& modules = instance().modules;
    auto iter = modules.find(key);
    if (iter == modules.end()) {
      iter = modules.emplace(key, std::make_unique<ModuleMap<Factory>>()).first;
    }

    auto derived = dynamic_cast<ModuleMap<Factory>*>(iter->second.get());
    if (!derived) {
      Logger::logFatal("Invalid module map for type '" + type + "' and info '" + key.type_info + "'");
      return;
    }

    std::cout << "Adding type '" << type << "' & info '" << key.type_info << "' @ " << derived << std::endl;
    if (!derived->addEntry(type, method)) {
      if (!key.type_info.empty()) {
        Logger::logError("Cannot register already existent type '" + type + "' for " + key.type_info + ".");
      }

      return;
    }

    auto& types = instance().type_registry[key];
    types.emplace_back(type, typeName<DerivedT>());
    std::sort(types.begin(), types.end());
  }

  template <typename BaseT, typename... Args>
  static FactoryMethod<BaseT, Args...> getModule(const std::string& type, const std::string& registration_info) {
    using Factory = FactoryMethod<BaseT, Args...>;
    const auto key = ModuleKey::fromTypes<BaseT, Args...>();
    const auto& modules = instance().modules;

    const auto iter = modules.find(key);
    if (iter == modules.end()) {
      Logger::logError("Cannot create a module of type '" + type + "': No modules registered to the factory for " +
                       key.type_info + ". Register modules using a static " + registration_info + " struct.\n" +
                       getAllRegistered());
      return {};
    }

    auto derived = dynamic_cast<ModuleMap<Factory>*>(iter->second.get());
    if (!derived) {
      Logger::logFatal("Invalid module map for type '" + type + "' and info '" + key.type_info + "'");
      return {};
    }

    const auto factory = derived->getEntry(type);
    if (!factory && !type.empty()) {
      Logger::logError("No module of type '" + type + "' registered to the factory for " + key.type_info +
                       ". Registered are: '" + getRegistered(key) + "'.");
    }

    return factory;
  }

  static std::string getAllRegistered();
  static std::string getRegistered(const ModuleKey& module);
  static void lock();
  static void unlock();
  static bool locked();

 private:
  static std::unique_ptr<ModuleRegistry> s_instance_;
  static ModuleRegistry& instance();
  ModuleRegistry() = default;

  bool locked_;

  std::map<ModuleKey, std::unique_ptr<ModuleMapBase>> modules;
  // Nested modules: base_type + args -> registered <type_name, type>.
  std::map<ModuleKey, std::vector<std::pair<std::string, std::string>>> type_registry;
};

// Helper function to read the type param from a node.
bool getTypeImpl(const YAML::Node& data, std::string& type, const std::string& param_name);

bool getType(const YAML::Node& data, std::string& type);

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

// Registry for config names based on derived types in the factory.
template <typename BaseT, typename ConfigT>
class ConfigTypeRegistry {
 public:
  static void setTypeName(const std::string& type) {
    // NOTE(lschmid): This is not forbidden behavior, but is not recommended so for now simply warn the user.
    std::string& type_ = instance().type_;
    if (!type_.empty() && type_ != type) {
      Logger::logInfo("Overwriting type name for config '" + typeName<ConfigT>() + "' for base module '" +
                      typeName<BaseT>() + "' from '" + instance().type_ + "' to '" + type +
                      "'. Defining different type identifiers for the same derived module is not recommended.");
    }
    type_ = type;
  }
  static std::string getType() { return instance().type_; }

 private:
  static ConfigTypeRegistry& instance() {
    static ConfigTypeRegistry instance_;
    return instance_;
  }

  ConfigTypeRegistry() = default;

  std::string type_;
};

// Definitions of the Factories.
// Factory to create configs.
template <class BaseT>
struct ConfigFactory {
  using Factory = FactoryMethod<ConfigWrapper>;
  inline static constexpr auto registration_info =
      "config::RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>";

  // Add entries.
  template <class DerivedConfigT>
  static void addEntry(const std::string& type) {
    const auto locked = ModuleRegistry::locked();
    const Factory method = [type, locked]() -> ConfigWrapper* {
      if (locked) {
        Logger::logWarning("external method!");
      }
      return new ConfigWrapperImpl<DerivedConfigT>(type);
    };

    // TODO(nathan be careful with extra type info conflicts, see below
    // If the config is already registered, e.g. from different constructor args no warning needs to be printed.
    // ModuleMap::addEntry(type, method, "");
    ModuleRegistry::addModule<ConfigWrapper, DerivedConfigT>(type, method);

    // Register the type name for the config.
    ConfigTypeRegistry<BaseT, DerivedConfigT>::setTypeName(type);
  }

  // Create the config.
  static std::unique_ptr<ConfigWrapper> create(const std::string& type) {
    const auto factory = ModuleRegistry::getModule<BaseT>(type, registration_info);
    if (!factory) {
      return nullptr;
    }

    // return std::unique_ptr<ConfigWrapper>(factory());
    return nullptr;
  }
};

// Factory to create DerivedT objects without configs.
template <class BaseT, typename... Args>
struct ObjectFactory {
  using Factory = FactoryMethod<BaseT, Args...>;
  inline static constexpr auto registration_info = "config::Registration<BaseT, DerivedT, ConstructorArguments...>";

  // Add entries.
  template <typename DerivedT>
  static void addEntry(const std::string& type) {
    const auto locked = ModuleRegistry::locked();
    const Factory method = [locked](Args... args) -> BaseT* {
      if (locked) {
        Logger::logWarning("external method!");
      }
      return new DerivedT(args...);
    };

    ModuleRegistry::addModule<BaseT, DerivedT, Args...>(type, method);
  }

  static std::unique_ptr<BaseT> create(const std::string& type, Args... args) {
    const auto factory = ModuleRegistry::getModule<BaseT, Args...>(type, registration_info);
    if (!factory) {
      return nullptr;
    }

    return std::unique_ptr<BaseT>(factory(args...));
  }
};

// Factory to create DerivedT objects that use configs.
template <class BaseT, typename... Args>
struct ObjectWithConfigFactory {
  using Factory = FactoryMethod<BaseT, const YAML::Node&, Args...>;
  inline static constexpr auto registration_info =
      "config::RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>";

  // Add entries.
  template <typename DerivedT, typename DerivedConfigT>
  static void addEntry(const std::string& type) {
    const auto locked = ModuleRegistry::locked();
    const Factory method = [locked](const YAML::Node& data, Args... args) -> BaseT* {
      if (locked) {
        Logger::logWarning("external method!");
      }

      DerivedConfigT config;
      Visitor::setValues(config, data);
      return new DerivedT(config, args...);
    };

    ModuleRegistry::addModule<BaseT, DerivedT, const YAML::Node&, Args...>(type, method);
  }

  static std::unique_ptr<BaseT> create(const YAML::Node& data, Args... args) {
    std::string type;
    if (!getType(data, type)) {
      return nullptr;
    }

    const auto factory = ModuleRegistry::getModule<BaseT, YAML::Node, Args...>(type, registration_info);
    if (!factory) {
      return nullptr;
    }

    return std::unique_ptr<BaseT>(factory(data, args...));
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
  return internal::ObjectFactory<BaseT, ConstructorArguments...>::create(type, args...);
}

}  // namespace config
