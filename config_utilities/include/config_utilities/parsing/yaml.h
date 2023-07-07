#pragma once

#include <fstream>
#include <memory>
#include <string>

#include <yaml-cpp/yaml.h>

#include "config_utilities/factory.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"

namespace config {

/**
 * @brief Loads a config from a yaml node.
 *
 * @tparam ConfigT The config type.
 * @param node The yaml node to load from.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 * Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromYaml(const YAML::Node& node, const std::string& name_space = "") {
  ConfigT config;
  internal::Visitor::setValues(config, internal::lookupNamespace(node, name_space));
  return config;
}

/**
 * @brief Saves a config to a yaml node.
 *
 * @tparam ConfigT The config type.
 * @param config The config to save.
 * @returns The yaml node.
 */
template <typename ConfigT>
YAML::Node toYaml(const ConfigT& config) {
  const internal::MetaData data = internal::Visitor::getValues(config);
  return data.data;
}

/**
 * @brief Loads a config from a yaml file.
 *
 * @tparam ConfigT The config type.
 * @param file_name The file name to load as full path.
 * @param name_space Optionally specify a name space to load from the yaml file. If empty, the whole file is loaded.
 * Separate names with slashes '/'. Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromYamlFile(const std::string& file_name, const std::string& name_space = "") {
  YAML::Node node = internal::lookupNamespace(YAML::LoadFile(file_name), name_space);
  ConfigT config;
  internal::Visitor::setValues(config, node);
  return config;
}

/**
 * @brief Saves a config to a yaml file.
 *
 * @tparam ConfigT The config type.
 * @param config The config to save.
 * @param file_name The file name to save to as full path including extension.
 * @returns True if the file was successfully saved. False if some error occured.
 */
template <typename ConfigT>
bool toYamlFile(const ConfigT& config, const std::string& file_name) {
  const internal::MetaData data = internal::Visitor::getValues(config);
  YAML::Emitter out;
  out << data.data;
  // TODO(lschmid): Here should probably be some verification of the output and of the target file to be created, as
  // well as extension handling.
  std::ofstream fout(file_name);
  fout << std::string(out.c_str);
  return true;
}

/**
 * @brief Create a derived type object based on a the data stored in a yaml node. All derived types need to be
 * registered to the factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct. They
 * need to implement a config as a public member struct named 'Config' and use the config as the first constructor
 * argument.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
 * different base-entry in the factory.
 * @param node Yaml node containing the type identifier as a param and the data to create the config.
 * @param args Other constructor arguments.
 * @param name_space Optionally specify a name space to create the object from. Separate names with
 * slashes '/'. Example: "my_config/my_sub_config".
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromYaml(const YAML::Node& node,
                                      ConstructorArguments... args,
                                      const std::string& name_space = "") {
  return internal::Factory::create<BaseT>(internal::lookupNamespace(node, name_space), args...);
}

/**
 * @brief Create a derived type object based on a the data stored in a yaml node. All derived types need to be
 * registered to the factory using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct. They
 * need to implement a config as a public member struct named 'Config' and use the config as the first constructor
 * argument.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
 * different base-entry in the factory.
 * @param file_name The file name to load as full path.
 * @param args Other constructor arguments.
 * @param name_space Optionally specify a name space to load from the yaml file. If empty, the whole file is loaded.
 * Separate names with slashes '/'. Example: "my_config/my_sub_config".
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromYamlFile(const std::string& file_name,
                                          ConstructorArguments... args,
                                          const std::string& name_space = "") {
  return internal::Factory::create<BaseT>(internal::lookupNamespace(YAML::LoadFile(file_name), name_space), args...);
}

}  // namespace config
