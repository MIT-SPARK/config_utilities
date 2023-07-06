#pragma once

#include <fstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"

namespace config {

// NOTE(lschmid): These could two could also be cut/moved internally to avoid the the public dependency on yaml-cpp?

/**
 * @brief Loads a config from a yaml node.
 *
 * @tparam ConfigT The config type.
 * @param node The yaml node to load from.
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromYaml(const YAML::Node& node) {
  ConfigT config;
  internal::Visitor::setValues(config, node);
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
 * @param name_space The name space to load from the yaml file. If empty, the whole file is loaded. Separate names with
 * slashes '/'. Example: "my_config/my_sub_config".
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

}  // namespace config
