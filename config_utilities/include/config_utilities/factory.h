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

class ModuleRegistry {
 public:
  template <typename BaseT, typename DerivedT, typename... Args>
  static void addModule(const std::string& type) {
    const std::string base_type = typeName<BaseT>();
    const std::string derived_type = typeName<DerivedT>();
    std::stringstream ss;
    ((ss << typeName<Args>() << ", "), ...);
    std::string arguments = ss.str();
    if (!arguments.empty()) {
      arguments = arguments.substr(0, arguments.size() - 2);
    }

    auto& types = instance().modules[base_type][arguments];
    types.emplace_back(type, derived_type);
    std::sort(types.begin(), types.end());
  }

  static std::string getAllRegistered() {
    std::stringstream ss;
    ss << "Modules registered to factories: {";
    for (auto&& [base_type, args] : instance().modules) {
      for (auto&& [arguments, types] : args) {
        ss << "\n  " << base_type << "(" << arguments << "): {";
        for (auto&& [type_name, derived_type] : types) {
          ss << "\n    '" << type_name << "' (" << derived_type << "),";
        }
        ss << "\n  },";
      }
    }
    ss << "\n}";
    return ss.str();
  }

 private:
  static ModuleRegistry& instance() {
    static ModuleRegistry instance_;
    return instance_;
  }

  ModuleRegistry() = default;

  // Nested modules: base_type -> args -> registered <type_name, tpye>.
  std::map<std::string, std::map<std::string, std::vector<std::pair<std::string, std::string>>>> modules;
};

// Struct to store the factory methods for the creation of modules.
template <typename FactoryMethod>
struct ModuleMapBase {
  using FactoryMethodMap = std::unordered_map<std::string, FactoryMethod>;

  // Add entries to the map with verbose warnings.
  static bool addEntry(const std::string& type, const FactoryMethod& method, const std::string& type_info) {
    FactoryMethodMap& map = instance().map;
    if (map.find(type) != map.end()) {
      if (!type_info.empty()) {
        Logger::logError("Cannot register already existent type '" + type + "' for " + type_info + ".");
      }
      return false;
    }

    map.insert(std::make_pair(type, method));
    return true;
  }

  // Check if a requested type is valid with verbose warnings.
  static bool hasEntry(const std::string& type, const std::string& type_info, const std::string& registration_info) {
    FactoryMethodMap& map = instance().map;
    if (map.empty()) {
      Logger::logError("Cannot create a module of type '" + type + "': No modules registered to the factory for " +
                       type_info + ". Register modules using a static " + registration_info + " struct.\n" +
                       ModuleRegistry::getAllRegistered());
      return false;
    }
    if (map.find(type) == map.end()) {
      if (!type.empty()) {
        std::string module_list;
        for (const auto& entry : map) {
          module_list.append(entry.first + "', '");
        }
        module_list = module_list.substr(0, module_list.size() - 4);
        Logger::logError("No module of type '" + type + "' registered to the factory for " + type_info +
                         ". Registered are: '" + module_list + "'.");
      }
      return false;
    }
    return true;
  }

  // Get the factory method for a type. This assumes that the type is valid.
  static FactoryMethod getEntry(const std::string& type) { return instance().map[type]; }

 private:
  // Constructor.
  ModuleMapBase() = default;

  // Singleton access.
  static ModuleMapBase& instance() {
    static ModuleMapBase instance_;
    return instance_;
  }

  // The map.
  FactoryMethodMap map;
};

// Helper function to get human readable type infos.
template <class BaseT, typename... Args>
inline std::string typeInfo() {
  std::string type_info = "BaseT='" + typeName<BaseT>() + "' and ConstructorArguments={";
  std::stringstream ss;
  ((ss << typeName<Args>() << "', '"), ...);
  const std::string arguments = ss.str();
  if (!arguments.empty()) {
    type_info += "'" + arguments.substr(0, arguments.size() - 3);
  }
  type_info += "}";
  return type_info;
}

// Helper function to read the type param from a node.
inline bool getTypeImpl(const YAML::Node& data, std::string& type, const std::string& param_name) {
  if (!data.IsMap()) {
    return false;
  }
  if (!data[param_name]) {
    return false;
  }
  try {
    type = data[param_name].as<std::string>();
  } catch (const YAML::Exception& e) {
    return false;
  }
  return true;
}

inline bool getType(const YAML::Node& data, std::string& type) {
  // Get the type or print an error.
  const std::string param_name = Settings::instance().factory_type_param_name;
  if (!getTypeImpl(data, type, param_name)) {
    Logger::logError("Could not read the param '" + param_name + "' to deduce the type of the module to create.");
    return false;
  }
  return true;
}

// Wrapper struct for any config type.
struct ConfigWrapper {
  explicit ConfigWrapper(const std::string& _type) : type(_type) {}
  virtual ~ConfigWrapper() = default;
  virtual std::unique_ptr<ConfigWrapper> clone() const = 0;
  virtual void onDeclareConfig() = 0;
  virtual std::unique_ptr<ConfigWrapper> createDefault() const = 0;
  std::string type;
};

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
  using FactoryMethod = std::function<ConfigWrapper*()>;
  using ModuleMap = ModuleMapBase<FactoryMethod>;

  // Add entries.
  template <class DerivedConfigT>
  static void addEntry(const std::string& type) {
    FactoryMethod method = [type]() { return new ConfigWrapperImpl<DerivedConfigT>(type); };
    // If the config is already registered, e.g. from different constructor args no warning needs to be printed.
    ModuleMap::addEntry(type, method, "");

    // Register the type name for the config.
    ConfigTypeRegistry<BaseT, DerivedConfigT>::setTypeName(type);
  }

  // Create the config.
  static std::unique_ptr<ConfigWrapper> create(const std::string& type) {
    if (ModuleMap::hasEntry(
            type,
            "BaseT='" + typeName<BaseT>() + "'",
            "config::RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>")) {
      const FactoryMethod creation_method = ModuleMap::getEntry(type);
      return std::unique_ptr<ConfigWrapper>(creation_method());
    }
    return nullptr;
  }
};

// Factory to create DerivedT objects without configs.
template <class BaseT, typename... Args>
struct ObjectFactory {
  using FactoryMethod = std::function<BaseT*(Args...)>;
  using ModuleMap = ModuleMapBase<FactoryMethod>;

  // Add entries.
  template <typename DerivedT>
  static void addEntry(const std::string& type) {
    FactoryMethod method = [](Args... args) { return new DerivedT(args...); };
    if (ModuleMap::addEntry(type, method, typeInfo<BaseT, Args...>())) {
      ModuleRegistry::addModule<BaseT, DerivedT, Args...>(type);
    }
  }

  static std::unique_ptr<BaseT> create(const std::string& type, Args... args) {
    if (ModuleMap::hasEntry(
            type, typeInfo<BaseT, Args...>(), "config::Registration<BaseT, DerivedT, ConstructorArguments...>")) {
      const FactoryMethod creation_method = ModuleMap::getEntry(type);
      return std::unique_ptr<BaseT>(creation_method(args...));
    }
    return nullptr;
  }
};

// Factory to create DerivedT objects that use configs.
template <class BaseT, typename... Args>
struct ObjectWithConfigFactory {
  using FactoryMethod = std::function<BaseT*(const YAML::Node&, Args...)>;
  using ModuleMap = ModuleMapBase<FactoryMethod>;

  // Add entries.
  template <typename DerivedT, typename DerivedConfigT>
  static void addEntry(const std::string& type) {
    FactoryMethod method = [](const YAML::Node& data, Args... args) {
      DerivedConfigT config;
      Visitor::setValues(config, data);
      return new DerivedT(config, args...);
    };
    if (ModuleMap::addEntry(type, method, typeInfo<BaseT, Args...>())) {
      ModuleRegistry::addModule<BaseT, DerivedT, Args...>(type);
    }
  }

  static std::unique_ptr<BaseT> create(const YAML::Node& data, Args... args) {
    std::string type;
    if (!getType(data, type)) {
      return nullptr;
    }
    if (ModuleMap::hasEntry(
            type,
            typeInfo<BaseT, Args...>(),
            "config::RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>")) {
      const FactoryMethod creation_method = ModuleMap::getEntry(type);

      return std::unique_ptr<BaseT>(creation_method(data, args...));
    }
    return nullptr;
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
