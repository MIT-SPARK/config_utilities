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

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/node/node.h>
#include <yaml-cpp/yaml.h>

#include "config_utilities/factory.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"

namespace config {

/**
 * @brief Loads a config from a yaml node.
 *
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param node The yaml node to load from.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 * Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromYaml(const YAML::Node& node, const std::string& name_space = "") {
  ConfigT config;
  internal::Visitor::setValues(config, internal::lookupNamespace(node, name_space), true);
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
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param file_name The file name to load as full path.
 * @param name_space Optionally specify a name space to load from the yaml file. If empty, the whole file is loaded.
 * Separate names with slashes '/'. Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromYamlFile(const std::string& file_name, const std::string& name_space = "") {
  const YAML::Node node = internal::lookupNamespace(YAML::LoadFile(file_name), name_space);
  return fromYaml<ConfigT>(node, "");
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
  // well as extension handling. For now let ofstream handle it.
  std::ofstream fout(file_name);
  fout << std::string(out.c_str());
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
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromYaml(const YAML::Node& node, ConstructorArguments... args) {
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(node, args...);
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
 * @param name_space Optionally specify a name space to create the object from. Separate names with
 * slashes '/'. Example: "my_config/my_sub_config".
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromYamlWithNamespace(const YAML::Node& node,
                                                   const std::string& name_space,
                                                   ConstructorArguments... args) {
  const YAML::Node ns_node = internal::lookupNamespace(node, name_space);
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(ns_node, args...);
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
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromYamlFile(const std::string& file_name, ConstructorArguments... args) {
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(YAML::LoadFile(file_name), args...);
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
 * @param name_space Optionally specify a name space to load from the yaml file. If empty, the whole file is loaded.
 * Separate names with slashes '/'. Example: "my_config/my_sub_config".
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromYamlFileWithNamespace(const std::string& file_name,
                                                       const std::string& name_space,
                                                       ConstructorArguments... args) {
  const YAML::Node node = internal::lookupNamespace(YAML::LoadFile(file_name), name_space);
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(node, args...);
}

/**
 * @brief Update the config with the values in a YAML node.
 * @note This function will update the field and check the validity of the config afterwards. If the config is invalid,
 * the field will be reset to its original value.
 * @param config The config to update.
 * @param node The node containing the field(s) to update.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 */
template <typename ConfigT>
bool updateFromYaml(ConfigT& config, const YAML::Node& node, const std::string& name_space = "") {
  return updateField(config, node, true, name_space);
}

}  // namespace config
