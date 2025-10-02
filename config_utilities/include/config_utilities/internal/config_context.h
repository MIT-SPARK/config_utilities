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

#include <yaml-cpp/yaml.h>

#include "config_utilities/factory.h"
#include "config_utilities/internal/introspection.h"
#include "config_utilities/internal/visitor.h"

// TMP
#include <iostream>

namespace config::internal {

/**
 * @brief Context is a singleton that holds the raw parsed information used to generate configs
 */
class Context {
 public:
  ~Context() = default;

  /**
   * @brief Update the context by merging in a new YAML node.
   * @param other The node to merge into the context.
   * @param ns Optional namespace to move the node down into before merging.
   * @param merge_mode The merge mode to use when merging the new node into the existing context.
   * @param by If provided, the merge will be logged as an introspection event with this source.
   */
  static void update(const YAML::Node& other,
                     const std::string& ns,
                     internal::MergeMode merge_mode,
                     Introspection::By* by = nullptr);

  static void clear();

  static YAML::Node toYaml();

  template <typename BaseT, typename... ConstructorArguments>
  static std::unique_ptr<BaseT> create(ConstructorArguments... args) {
    return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(instance().contents_,
                                                                                     std::move(args)...);
  }

  template <typename BaseT, typename... ConstructorArguments>
  static std::unique_ptr<BaseT> createNamespaced(const std::string& name_space, ConstructorArguments... args) {
    const auto ns_node = internal::lookupNamespace(instance().contents_, name_space);
    if (!Settings::instance().introspection.enabled()) {
      return internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(ns_node, std::move(args)...);
    }
    // Log introspection at the correct namespace.
    Introspection::enterNamespace(name_space);
    auto obj = internal::ObjectWithConfigFactory<BaseT, ConstructorArguments...>::create(ns_node, std::move(args)...);
    Introspection::exitNamespace();
    return obj;
  }

  template <typename ConfigT>
  static ConfigT loadConfig(const std::string& name_space = "") {
    ConfigT config;
    const auto set_data = internal::Visitor::setValues(config, instance().contents_, true, name_space);
    if (internal::Settings::instance().introspection.enabled()) {
      const auto get_info = internal::Visitor::getInfo(config, name_space);
      Introspection::logSetValue(set_data, get_info);
    }
    return config;
  }

 private:
  Context() = default;
  static Context& instance();

  YAML::Node contents_;
};

}  // namespace config::internal
