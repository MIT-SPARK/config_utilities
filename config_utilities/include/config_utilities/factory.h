#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config {

namespace internal {

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
                       type_info + ". Register modules using a static " + registration_info + " struct.");
      return false;
    }
    if (map.find(type) == map.end()) {
      std::string module_list;
      for (const auto& entry : map) {
        module_list.append(entry.first + "', '");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      Logger::logError("No module of type '" + type + "' registered to the factory for " + type_info +
                       ". Registered are: '" + module_list + "'.");
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
  std::string type_info = "BaseT='" + std::string(typeid(BaseT).name()) + "' and ConstructorArguments={";
  std::stringstream ss;
  ((ss << typeid(Args).name() << "', '"), ...);
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
    type = data[Settings::instance().factory_type_param_name].as<std::string>();
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
  void onDeclareConfig() { declare_config(config); }
  std::unique_ptr<ConfigWrapper> createDefault() const override {
    return std::make_unique<ConfigWrapperImpl<ConfigT>>(type);
  };
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
  }

  // Create the config.
  static std::unique_ptr<ConfigWrapper> create(const std::string& type) {
    if (ModuleMap::hasEntry(
            type,
            "BaseT='" + std::string(typeid(BaseT).name()) + "'",
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
    ModuleMap::addEntry(type, method, typeInfo<BaseT, Args...>());
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
    ModuleMap::addEntry(type, method, typeInfo<BaseT, Args...>());
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
