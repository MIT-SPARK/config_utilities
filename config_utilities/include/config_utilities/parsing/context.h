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

#include <string>

#include "config_utilities/internal/config_context.h"

namespace config {

/**
 * @brief Initialize global config context from the command line
 * @param argc Number of arguments.
 * @param argv Actual command line arguments.
 * @param remove_arguments Remove parsed command line arguments.
 */
void initContext(int& argc, char* argv[], bool remove_arguments = true);

/**
 * @brief Aggregate YAML node into global context
 * @param node YAML to add to context
 * @param ns Optional namespace
 * @param merge_mode Merge mode to use when merging the new node into the existing context. The default is APPEND, which
 * will act like the ROS1 param server and extend sequences
 */
void pushToContext(const YAML::Node& node,
                   const std::string& ns = "",
                   internal::MergeMode merge_mode = internal::MergeMode::APPEND);

/**
 * @brief Delete parsed context
 */
void clearContext();

/**
 * @brief Dump current context for exporting or saving
 */
YAML::Node contextToYaml();

/**
 * @brief Loads a config from the global context
 *
 * @tparam ConfigT The config type. This can also be a VirtualConfig<BaseT> or a std::vector<ConfigT>.
 * @param name_space Optionally specify a name space to create the config from. Separate names with slashes '/'.
 * Example: "my_config/my_sub_config".
 * @returns The config.
 */
template <typename ConfigT>
ConfigT fromContext(const std::string& name_space = "") {
  return internal::Context::loadConfig<ConfigT>(name_space);
}

/**
 * @brief Create a derived type object based on currently stored YAML in config::internal::Context.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments. Note that each unique set of constructor arguments will result in a
 * different base-entry in the factory.
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromContext(ConstructorArguments... args) {
  return internal::Context::create<BaseT, ConstructorArguments...>(std::move(args)...);
}

/**
 * @brief Create a derived type object based on currently stored YAML in config::internal::Context.
 *
 * See createFromYamlWithNamespace() for more specific behavioral information.
 *
 * @tparam BaseT Type of the base class to be constructed.
 * @tparam Args Other constructor arguments.
 * @param name_space Optionally specify a name space to create the object from.
 * @param args Other constructor arguments.
 * @returns Unique pointer of type base that contains the derived object.
 */
template <typename BaseT, typename... ConstructorArguments>
std::unique_ptr<BaseT> createFromContextWithNamespace(const std::string& name_space, ConstructorArguments... args) {
  return internal::Context::createNamespaced<BaseT, ConstructorArguments...>(name_space, args...);
}

/**
 * @brief Load global settings for `config_utilities` from current parsed context
 * @param name_space Namespace to load the settings from
 */
void setConfigSettingsFromContext(const std::string& name_space = "");

}  // namespace config
