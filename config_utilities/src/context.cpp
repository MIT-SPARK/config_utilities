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

#include "config_utilities/parsing/context.h"

#include <memory>

#include "config_utilities/internal/config_context.h"
#include "config_utilities/internal/introspection.h"
#include "config_utilities/parsing/commandline.h"
#include "config_utilities/settings.h"
#include "config_utilities/update.h"

namespace config {

void initContext(int& argc, char* argv[], bool remove_arguments) {
  const auto node = internal::loadFromArguments(argc, argv, remove_arguments);
  internal::Context::update(node, "", internal::MergeMode::APPEND);
}

void pushToContext(const YAML::Node& node, const std::string& ns, internal::MergeMode merge_mode) {
  std::unique_ptr<internal::Introspection::By> by = nullptr;
  if (Settings().introspection.enabled()) {
    by = std::make_unique<internal::Introspection::By>(
        internal::Introspection::By::programmatic("pushToContext()" + (ns.empty() ? "" : "@" + ns)));
  }
  internal::Context::update(node, ns, merge_mode, by.get());
}

void clearContext() {
  if (Settings().introspection.enabled()) {
    internal::Introspection::logClear(internal::Introspection::By::programmatic("clearContext()"));
  }
  internal::Context::clear();
}

YAML::Node contextToYaml() { return internal::Context::toYaml(); }

void setConfigSettingsFromContext(const std::string& name_space) {
  const auto node = internal::Context::toYaml();
  const auto set_data = internal::Visitor::setValues(Settings(), internal::lookupNamespace(node, name_space), true);
  if (Settings().introspection.enabled()) {
    const auto get_info = internal::Visitor::getInfo(Settings(), name_space);
    internal::Introspection::logSetValue(set_data, get_info);
  }
}

}  // namespace config
