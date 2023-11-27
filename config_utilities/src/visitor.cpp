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

#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/settings.h"

namespace config::internal {

thread_local std::vector<Visitor*> Visitor::instances = {};

Visitor::~Visitor() { instances.pop_back(); }

bool Visitor::hasInstance() { return !instances.empty(); }

Visitor::Visitor(Mode _mode, const std::string& _name_space, const std::string& field_name)
    : mode(_mode), name_space(_name_space) {
  data.field_name = field_name;
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
  std::string& current_name = Visitor::instance().data.name;
  // Avoid overriding names. If the name is to be set, it should be cleared previously.
  if (current_name.empty()) {
    current_name = name;
  }
}

void Visitor::visitCheck(const CheckBase& check) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.data.checks.emplace_back(check.clone());
}

std::optional<YAML::Node> Visitor::visitVirtualConfig(bool is_set, bool is_optional, const std::string& type) {
  Visitor& visitor = Visitor::instance();
  visitor.data.is_virtual_config = true;

  // Treat the validity of virtual configs as checks.
  if (visitor.mode == Visitor::Mode::kCheck) {
    if (!is_set && !is_optional) {
      const std::string field_name = visitor.data.field_name.empty() ? "" : "'" + visitor.data.field_name + "' ";
      visitor.data.checks.emplace_back(
          new Check(false, "Virtual config " + field_name + "is not set and not marked optional"));
    } else {
      visitor.data.checks.emplace_back(new Check(true, ""));
    }
  }

  if (visitor.mode == Visitor::Mode::kGet) {
    if (is_set) {
      // Also write the type param back to file.
      std::string error;
      YAML::Node type_node =
          YamlParser::toYaml(Settings::instance().factory_type_param_name, type, visitor.name_space, error);
      mergeYamlNodes(visitor.data.data, type_node);
    }
  }

  if (visitor.mode == Visitor::Mode::kSet) {
    // Return the data to intialize the virtual config if this is the first time setting it.
    return lookupNamespace(visitor.data.data, visitor.name_space);
  }

  return std::nullopt;
}

}  // namespace config::internal
