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

#include "config_utilities/internal/visitor.h"

#include <stdexcept>

#include "config_utilities/factory.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/settings.h"

namespace config::internal {

thread_local std::vector<Visitor*> Visitor::instances = {};

Visitor::~Visitor() { instances.pop_back(); }

bool Visitor::hasInstance() { return !instances.empty(); }

Visitor::Visitor(Mode _mode, const std::string& _name_space, const std::string& field_name)
    : mode(_mode), name_space(_name_space) {
  meta_data.field_name = field_name;
  meta_data.ns = name_space;
  // Create instances in a stack per thread and store the reference to it.
  instances.emplace_back(this);
}

Visitor& Visitor::instance() {
  if (instances.empty()) {
    // This should never happen as visitors are managed internally. Caught here for debugging.
    throw std::runtime_error(
        "Visitor instance was accessed but no visitor was created before. Visitor::instance() should only be called "
        "from within a visitor.");
  }
  return *instances.back();
}

void Visitor::visitName(const std::string& name) {
  std::string& current_name = Visitor::instance().meta_data.name;
  // Avoid overriding names. If the name is to be set, it should be cleared previously.
  if (current_name.empty()) {
    current_name = name;
  }
}

void Visitor::visitCheck(const CheckBase& check) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode == Visitor::Mode::kCheck || visitor.mode == Visitor::Mode::kGetInfo) {
    visitor.meta_data.checks.emplace_back(check.clone());
  }
}

std::optional<std::string> Visitor::visitVirtualConfig(bool is_set,
                                                       bool is_optional,
                                                       const std::string& type,
                                                       const std::string& base_type,
                                                       const std::string& base_factory_info) {
  Visitor& visitor = Visitor::instance();
  visitor.meta_data.virtual_config_type = type;

  // Treat the validity of virtual configs as checks.
  if (visitor.mode == Visitor::Mode::kCheck) {
    if (!is_set && !is_optional) {
      const std::string field_name =
          visitor.meta_data.field_name.empty() ? "" : "'" + visitor.meta_data.field_name + "' ";
      visitor.meta_data.checks.emplace_back(
          new Check(false, "Virtual config " + field_name + "is not set and not marked optional"));
    } else {
      visitor.meta_data.checks.emplace_back(new Check(true, ""));
    }
  }

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetInfo) {
    // Also write the type param back to file.
    auto type_node = YAML::Node(is_set ? type : kUninitializedVirtualConfigType);
    const std::string ns = joinNamespace(visitor.name_space, Settings::instance().factory.type_param_name);
    moveDownNamespace(type_node, ns);
    mergeYamlNodes(visitor.meta_data.data, type_node);
    auto& field_info = visitor.meta_data.field_infos.emplace_back();
    field_info.name = Settings::instance().factory.type_param_name;
    field_info.ns = visitor.additionalNamespace();
    field_info.value = lookupNamespace(visitor.meta_data.data, ns);
    field_info.is_meta_field = true;
  }

  if (visitor.mode == internal::Visitor::Mode::kGetInfo) {
    visitor.meta_data.available_types = ModuleRegistry::getRegisteredConfigTypes(base_type);
    if (is_optional) {
      visitor.meta_data.available_types.push_back(kUninitializedVirtualConfigType);
    }
  }

  if (visitor.mode == Visitor::Mode::kSet) {
    auto& field_info = visitor.meta_data.field_infos.emplace_back();
    field_info.name = internal::Settings::instance().factory.type_param_name;
    field_info.ns = visitor.additionalNamespace();
    field_info.is_meta_field = true;
    const YAML::Node type_node =
        lookupNamespace(visitor.meta_data.data, visitor.name_space);  // Including the param name.
    field_info.value = lookupNamespace(type_node, field_info.name);

    // underlying derived type is not required if the config is optional, or if the config has been
    // initialized to a derived type already.
    const bool type_required = !is_set && !is_optional;
    std::string type_param;
    if (internal::getType(type_node, type_param, type_required)) {
      field_info.was_parsed = true;
      return type_param;
    } else if (type_required) {
      internal::Logger::logError("Could not get type for '" + base_factory_info + "'");
    }
  }

  return std::nullopt;
}

std::string Visitor::additionalNamespace() const {
  const auto& meta_ns = meta_data.ns;
  if (name_space.length() > meta_ns.length()) {
    return name_space.substr(meta_ns.length() + 1);
  }
  return "";
}

}  // namespace config::internal
