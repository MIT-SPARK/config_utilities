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

#include <memory>
#include <string>

#include "config_utilities/factory.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"

namespace config {
namespace internal {

struct ParserInfo {
  bool help_present = false;
  bool force_block_style = false;
};

/**
 * @brief Parse and collate YAML node from arguments, optionally removing arguments
 * @param argc Number of command line arguments
 * @param argv Command line argument strings
 * @param remove_args Remove any recognized arguments from argc and argv
 * @param parser_info Optional information for parser used by composite executable
 */
YAML::Node loadFromArguments(int& argc, char* argv[], bool remove_args, ParserInfo* parser_info = nullptr);

/**
 * @brief Parse and collate YAML node from arguments
 * @param args Vector of command line argument strings
 */
YAML::Node loadFromArguments(const std::vector<std::string>& args);

}  // namespace internal

/**
 * @brief Loads a config based on collated YAML data specified via the command line
 *
 * See fromYaml() for more specific behavioral information.
 *
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param argc Number of arguments.
 * @param argv Actual command line arguments.
 * @param name_space Optional namespace to use under the resolved YAML parameter tree.
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromCLI(int argc, char* argv[], const std::string& name_space = "") {
  // when parsing CLI locally we don't want to modify the arguments ever
  const auto node = internal::loadFromArguments(argc, argv, false);

  ConfigT config;
  internal::Visitor::setValues(config, internal::lookupNamespace(node, name_space), true);
  return config;
}

/**
 * @brief Loads a config based on collated YAML data specified via the command line
 *
 * See fromYaml() for more specific behavioral information.
 *
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param argc Vector of command line arguments.
 * @param name_space Optional namespace to use under the resolved YAML parameter tree.
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromCLI(const std::vector<std::string>& args, const std::string& name_space = "") {
  const auto node = internal::loadFromArguments(args);

  ConfigT config;
  internal::Visitor::setValues(config, internal::lookupNamespace(node, name_space), true);
  return config;
}

/**
 * @brief Create a derived type object based on collated YAML data specified via the command line
 *
 * See createFromYaml() for more specific behavioral information.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments.
 * @param argc Number of arguments.
 * @param argv Actual command line arguments.
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromCLI(int argc, char* argv[], ConstructorArguments... args) {
  // when parsing CLI locally we don't want to modify the arguments ever
  const auto node = internal::loadFromArguments(argc, argv, false);
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(node, std::move(args)...);
}

/**
 * @brief Create a derived type object based on collated YAML data specified via the command line
 *
 * See createFromYaml() for more specific behavioral information.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments.
 * @param argv Command line arguments
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromCLI(const std::vector<std::string>& argv, ConstructorArguments... args) {
  const auto node = internal::loadFromArguments(argv);
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(node, std::move(args)...);
}

/**
 * @brief Create a derived type object based on collated YAML data specified via the command line
 *
 * See createFromYamlWithNamespace() for more specific behavioral information.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments.
 * @param argc Number of arguments.
 * @param argv Actual command line arguments.
 * @param name_space Optionally specify a name space to create the object from.
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromCLIWithNamespace(int argc,
                                                  char* argv[],
                                                  const std::string& name_space,
                                                  ConstructorArguments... args) {
  // when parsing CLI locally we don't want to modify the arguments ever
  const auto node = internal::loadFromArguments(argc, argv, false);
  const auto ns_node = internal::lookupNamespace(node, name_space);
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(ns_node, std::move(args)...);
}

/**
 * @brief Create a derived type object based on collated YAML data specified via the command line
 *
 * See createFromYamlWithNamespace() for more specific behavioral information.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments.
 * @param argv Vector of command line arguments.
 * @param name_space Optionally specify a name space to create the object from.
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromCLIWithNamespace(const std::vector<std::string>& argv,
                                                  const std::string& name_space,
                                                  ConstructorArguments... args) {
  const auto node = internal::loadFromArguments(argv);
  const auto ns_node = internal::lookupNamespace(node, name_space);
  return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(ns_node, std::move(args)...);
}

/**
 * @brief Create a derived type object based on collated YAML data specified via the command line
 * @param argv Vector of command line arguments
 * @param name_space Optional namespace to use for parsing settings
 */
void setConfigSettingsFromCLI(const std::vector<std::string>& argv, const std::string& name_space = "");

/**
 * @brief Create a derived type object based on collated YAML data specified via the command line
 * @param argc Command line argument count
 * @param argv Command line arguments
 * @param name_space Optional namespace to use for parsing settings
 */
void setConfigSettingsFromCLI(int argc, char* argv[], const std::string& name_space);

}  // namespace config
