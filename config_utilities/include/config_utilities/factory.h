#pragma once

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

#include "config_utilities/internal/logger.h"

namespace config {

namespace internal {

// Module map registering all structs for factory creation.
template <class BaseT, typename... Args>
struct ModuleMap {
 public:
  using FactoryMethod = std::function<BaseT*(Args... args)>;

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
      Logger::defaultLogger().logError(ss.str());
    } else {
      map.insert(std::make_pair(type, [](Args... args) { return new DerivedT(args...); }));
    }
  }
  // The map.
  std::unordered_map<std::string, FactoryMethod> map;

 private:
  ModuleMap() = default;
};

}  // namespace internal

/**
 * @brief Allocate these structs statically do add entries to the factory for run-time creation of modules based on
 * string identifiers.
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
    ((ss << typeid(args).name() << ", "), ...);
    std::string type_info = ss.str();
    if (!type_info.empty()) {
      type_info = " and ConstructorArguments={'" + type_info.substr(0, type_info.size() - 2) + "'}";
    } else {
      type_info = "";
    }
    if (module.map.empty()) {
      std::stringstream ss;
      ss << "Cannot create a module of type '" << type << "': No modules registered to the factory for BaseT='"
         << typeid(BaseT).name() << "'" << type_info
         << ". Register modules using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct.";
      Logger::defaultLogger().logError(ss.str());
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
      Logger::defaultLogger().logError(ss.str());
      return nullptr;
    }
    return std::unique_ptr<BaseT>(it->second(args...));
  }
};

}  // namespace internal

// Nicely wrapped access calls in the config namespace.
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
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> create(const std::string& type, ConstructorArguments... args) {
  return internal::Factory::create<BaseT>(type, args...);
}

}  // namespace config
