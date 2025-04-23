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

#include "config_utilities/substitutions.h"

#include <cstdlib>
#include <sstream>
#include <regex>

#include "config_utilities/internal/logger.h"

namespace config {
namespace {

static const auto env_reg = RegisteredSubstitutions::Registration<EnvSubstitution>();

inline void resolveSubstitution(YAML::Node node) {
  const auto tag = node.Tag();
  if (tag == "!append" || tag == "!update" || tag == "!replace") {
    node.SetTag("");
  }

  if (!node.IsScalar()) {
    return;
  }

  std::string result;

  bool has_match = false;
  auto to_search = node.as<std::string>();
  std::regex regex(R"""($\((\w+) (.+)\))""");
  for (std::smatch m; std::regex_search(to_search, m, regex);) {
    const std::string subs_name = m[1];
    auto parser = RegisteredSubstitutions::getEntry(subs_name);
    if (!parser) {
      internal::Logger::logWarning("Unknown substitution '" + subs_name + "'!");
      continue;
    }

    result += m.prefix();
    result += parser->process(m[2]);
    to_search = m.suffix();
  }

  // TODO(nathan) handle unmatched suffix

  if (!has_match) {
    return;
  }

  node = result;
}

}  // namespace

std::unique_ptr<RegisteredSubstitutions> RegisteredSubstitutions::s_instance_ = nullptr;

RegisteredSubstitutions::RegisteredSubstitutions() = default;

const Substitution* RegisteredSubstitutions::getEntry(const std::string& tag) {
  auto& curr = instance();
  auto iter = curr.entries_.find(tag);
  if (iter == curr.entries_.end()) {
    return nullptr;
  } else {
    return iter->second.get();
  }
}

RegisteredSubstitutions& RegisteredSubstitutions::instance() {
  if (!s_instance_) {
    s_instance_.reset(new RegisteredSubstitutions());
  }

  return *s_instance_;
}

void RegisteredSubstitutions::addEntry(const std::string& tag, std::unique_ptr<Substitution>&& proc) {
  auto& curr = instance();
  auto had_prev = curr.entries_.emplace(tag, std::move(proc)).second;
  if (!had_prev) {
    internal::Logger::logWarning("Dropping new processor for existing tag '" + tag + "'");
  }
}

std::string EnvSubstitution::process(const std::string& contents) const {
  const auto ret = std::getenv(contents.c_str());
  if (!ret) {
    std::stringstream ss;
    ss << "Failed to get envname from '" << contents << "'";
    internal::Logger::logWarning(ss.str());
    return contents;
  }

  return std::string(ret);
}

void resolveSubstitutions(YAML::Node node) {
  resolveSubstitution(node);
  switch (node.Type()) {
    case YAML::NodeType::Map:
      for (const auto& child : node) {
        // technically keys can have tags...
        // shouldn't need to recurse
        resolveSubstitution(child.first);
        // dispatch recursion to value
        resolveSubstitutions(child.second);
      }
      break;
    case YAML::NodeType::Sequence:
      // dispatch resolution to all children
      for (const auto& child : node) {
        resolveSubstitutions(child);
      }
      break;
    case YAML::NodeType::Undefined:
    case YAML::NodeType::Null:
    case YAML::NodeType::Scalar:
    default:
      return;
  }
}

}  // namespace config
