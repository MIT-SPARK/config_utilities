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

// Module map registering all structs for factory creation.
template <class BaseT, typename... Args>
struct ModuleMap {
 public:
  using FactoryMethod = std::function<BaseT*(Args... args)>;
  using FactoryWithConfigMethod = std::function<BaseT*(const YAML::Node&, Args... args)>;

  // Singleton access.
  static ModuleMap& instance() {
    static ModuleMap instance_;
    return instance_;
  }

  // Add entries.
  template <class DerivedT>
  void addEntry(const std::string& type) {
    if (map.find(type) != map.end()) {
      std::stringstream ss;
      ss << "Cannot register already existent type '" << type << "' for <DerivedT>='" << typeid(DerivedT).name()
         << "' to factory for base '" << typeid(BaseT).name() << "'.";
      Logger::logError(ss.str());
    } else {
      map.insert(std::make_pair(type, [](Args... args) { return new DerivedT(args...); }));
    }
  }

  template <class DerivedT, class DerivedConfigT>
  void addEntryWithConfig(const std::string& type) {
    static_assert(std::is_base_of<BaseT, DerivedT>::value, "DerivedT does not inherit from BaseT.");
    static_assert(isConfig<DerivedConfigT>(),
                  "DerivedConfigT is not a config. Please implement 'void declare_config(DerivedConfigT& config)'.");
    if (map_config.find(type) != map_config.end()) {
      std::stringstream ss;
      ss << "Cannot register already existent type '" << type << "' for <DerivedT>='" << typeid(DerivedT).name()
         << "' to factory for base '" << typeid(BaseT).name() << "'.";
      Logger::logError(ss.str());
      return;
    }
    map_config.insert(std::make_pair(type, [type](const YAML::Node& data, Args... args) -> BaseT* {
      DerivedConfigT config;
      Visitor::setValues(config, data);
      return new DerivedT(config, args...);
    }));
  }

  // The maps.
  std::unordered_map<std::string, FactoryMethod> map;
  std::unordered_map<std::string, FactoryWithConfigMethod> map_config;

 private:
  ModuleMap() = default;
};

// Wrapper struct for any config type.
struct ConfigWrapper {
  explicit ConfigWrapper(const std::string& _type) : type(_type) {}
  virtual ~ConfigWrapper() = default;
  virtual std::unique_ptr<ConfigWrapper> clone() = 0;
  virtual void onDeclareConfig() = 0;
  virtual MetaData getDefaultValues() const = 0;
  std::string type;
};

template <typename ConfigT>
struct ConfigWrapperImpl : public ConfigWrapper {
  explicit ConfigWrapperImpl(const std::string& _type) : ConfigWrapper(_type) {}
  ConfigT config;
  std::unique_ptr<ConfigWrapper> clone() override { return std::make_unique<ConfigWrapperImpl<ConfigT>>(*this); };
  void onDeclareConfig() { declare_config(config); }
  MetaData getDefaultValues() const override {
    ConfigT defaults;
    return internal::Visitor::getValues(defaults, false);
  };
};

template <class BaseT>
struct VirtualConfigModuleMap {
 public:
  using FactoryMethod = std::function<ConfigWrapper*()>;

  // Singleton access.
  static VirtualConfigModuleMap& instance() {
    static VirtualConfigModuleMap instance_;
    return instance_;
  }

  // Add entries.
  template <class DerivedConfigT>
  void addEntry(const std::string& type) {
    if (map.find(type) != map.end()) {
      // If the config is already registered, e.g. from different constructor args we don't need to do anything.
      return;
    }
    map.insert(
        std::make_pair(type, [type]() -> ConfigWrapper* { return new ConfigWrapperImpl<DerivedConfigT>(type); }));
  }

  // The maps.
  std::unordered_map<std::string, FactoryMethod> map;

 private:
  VirtualConfigModuleMap() = default;
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
    internal::ModuleMap<BaseT, ConstructorArguments...>::instance().template addEntry<DerivedT>(type);
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
    internal::ModuleMap<BaseT, ConstructorArguments...>::instance()
        .template addEntryWithConfig<DerivedT, DerivedConfigT>(type);
    internal::VirtualConfigModuleMap<BaseT>::instance().template addEntry<DerivedConfigT>(type);
  }
};

namespace internal {

class Factory {
 public:
  /**
   * @brief Query the factory to create a dervied type.
   *
   * @tparam BaseT Type of the base class to query for.
   * @tparam Args Other constructor arguments. Notice that each unique set of constructor arguments will result in a
   * different base-entry in the factory.
   * @param type String identifier of the derived type to create.
   * @param args Other constructor arguments.
   * @return std::unique_ptr<BaseT> Unique pointer of type base that contains the derived object.
   */
  template <class BaseT, typename... Args>
  static std::unique_ptr<BaseT> create(const std::string& type, Args... args) {
    ModuleMap<BaseT, Args...>& module = ModuleMap<BaseT, Args...>::instance();
    std::stringstream ss;
    ((ss << typeid(args).name() << "', '"), ...);
    std::string type_info = ss.str();
    if (!type_info.empty()) {
      type_info = " and ConstructorArguments={'" + type_info.substr(0, type_info.size() - 4) + "'}";
    } else {
      type_info = " and ConstructorArguments={}";
    }
    if (module.map.empty()) {
      std::stringstream ss;
      ss << "Cannot create a module of type '" << type << "': No modules registered to the factory for BaseT='"
         << typeid(BaseT).name() << "'" << type_info
         << ". Register modules using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct.";
      Logger::logError(ss.str());
      return nullptr;
    }
    auto it = module.map.find(type);
    if (it == module.map.end()) {
      std::string module_list;
      for (const auto& entry : module.map) {
        module_list.append(entry.first + ", ");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      std::stringstream ss;
      ss << "No module of type '" << type << "' registered to the factory for BaseT='" << typeid(BaseT).name() << "'"
         << type_info << ". Registered are: " << module_list << ".";
      Logger::logError(ss.str());
      return nullptr;
    }
    return std::unique_ptr<BaseT>(it->second(args...));
  }

  /**
   * @brief Query the factory to create a dervied typethat uses a config. Finds the type identifier in the provided data
   * to decide on the type and cosntruct the config and object.
   *
   * @tparam BaseT Type of the base class to query for.
   * @tparam Args Other constructor arguments. Notice that each unique set of constructor arguments will result in a
   * different base-entry in the factory.
   * @param data Data to lookup the type identifier and build the config.
   * @param args Other constructor arguments.
   * @return std::unique_ptr<BaseT> Unique pointer of type base that contains the derived object.
   */
  template <class BaseT, typename... Args>
  static std::unique_ptr<BaseT> createWithConfig(const YAML::Node& data, Args... args) {
    ModuleMap<BaseT, Args...>& module = ModuleMap<BaseT, Args...>::instance();
    std::stringstream ss;
    ((ss << typeid(args).name() << "', '"), ...);
    std::string type_info = ss.str();
    if (!type_info.empty()) {
      type_info = " and constructor arguments '" + type_info.substr(0, type_info.size() - 4) + "'";
    } else {
      type_info = " and ConstructorArguments={}";
    }

    // Get the type param.
    std::string type;
    try {
      type = YAML::Clone(data)[Settings::instance().factory_type_param_name].as<std::string>();
    } catch (const YAML::Exception& e) {
      std::stringstream ss;
      ss << "Could not read the param '" << Settings::instance().factory_type_param_name
         << "' to deduce the type of the module to create.";
      Logger::logError(ss.str());
      return nullptr;
    }

    // Check the source module exists.
    if (module.map_config.empty()) {
      std::stringstream ss;
      ss << "Cannot create a module of type '" << type << "': No modules registered to the factory for base '"
         << typeid(BaseT).name() << "'" << type_info
         << ". Register modules using a static config::RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, "
            "ConstructorArguments...> struct.";
      Logger::logError(ss.str());
      return nullptr;
    }

    auto it2 = module.map_config.find(type);
    if (it2 == module.map_config.end()) {
      std::string module_list;
      for (const auto& entry : module.map_config) {
        module_list.append(entry.first + ", ");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      std::stringstream ss;
      ss << "No module of type '" << type << "' registered to the factory for base '" << typeid(BaseT).name() << "'"
         << type_info << ". Registered are: " << module_list << ".";
      Logger::logError(ss.str());
      return nullptr;
    }

    // Get the config and create the target.
    return std::unique_ptr<BaseT>(it2->second(data, args...));
  }

  // Get a config wrapper for the module.
  template <class BaseT>
  static std::unique_ptr<ConfigWrapper> createConfig(const YAML::Node& data) {
    VirtualConfigModuleMap<BaseT>& module = VirtualConfigModuleMap<BaseT>::instance();

    // Get the type param.
    std::string type;
    try {
      type = YAML::Clone(data)[Settings::instance().factory_type_param_name].as<std::string>();
    } catch (const YAML::Exception& e) {
      std::stringstream ss;
      ss << "Could not read the param '" << Settings::instance().factory_type_param_name
         << "' to deduce the type of the virtual config to create.";
      Logger::logError(ss.str());
      return nullptr;
    }

    // Check the source module exists.
    if (module.map.empty()) {
      std::stringstream ss;
      ss << "Cannot create a virtual config of type '" << type << "': No modules registered to the factory for base '"
         << typeid(BaseT).name()
         << "'. Register modules using a static config::RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, "
            "ConstructorArguments...> struct.";
      Logger::logError(ss.str());
      return nullptr;
    }

    auto it = module.map.find(type);
    if (it == module.map.end()) {
      std::string module_list;
      for (const auto& entry : module.map) {
        module_list.append(entry.first + ", ");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      std::stringstream ss;
      ss << "No module of type '" << type << "' registered to the factory for base '" << typeid(BaseT).name()
         << "'. Registered are: " << module_list << ".";
      Logger::logError(ss.str());
      return nullptr;
    }
    return std::unique_ptr<ConfigWrapper>(it->second());
  }
};

}  // namespace internal

// Nicely wrapped access calls in the config namespace.
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
  return internal::Factory::create<BaseT>(type, args...);
}

}  // namespace config
