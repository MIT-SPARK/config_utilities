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

#include "config_utilities/tag_processors.h"

#include <cstdlib>
#include <sstream>

#include "config_utilities/internal/logger.h"

namespace config {
namespace {

static const auto env_registration = RegisteredTags::Registration<EnvTag>("!env");

inline void resolveTag(YAML::Node node) {
  const auto tag = node.Tag();
  if (tag == "!append" || tag == "!update" || tag == "!replace") {
    node.SetTag("");
    return;
  }

  auto parser = RegisteredTags::getEntry(tag);
  if (!parser) {
    return;
  }

  parser->processNode(node);
}

}  // namespace

std::unique_ptr<RegisteredTags> RegisteredTags::s_instance_ = nullptr;

RegisteredTags::RegisteredTags() = default;

const TagProcessor* RegisteredTags::getEntry(const std::string& tag) {
  auto& curr = instance();
  auto iter = curr.entries_.find(tag);
  if (iter == curr.entries_.end()) {
    return nullptr;
  } else {
    return iter->second.get();
  }
}

RegisteredTags& RegisteredTags::instance() {
  if (!s_instance_) {
    s_instance_.reset(new RegisteredTags());
  }

  return *s_instance_;
}

void RegisteredTags::addEntry(const std::string& tag, std::unique_ptr<TagProcessor>&& proc) {
  auto& curr = instance();
  auto had_prev = curr.entries_.emplace(tag, std::move(proc)).second;
  if (had_prev) {
    internal::Logger::logWarning("Registered new processor for existing tag '" + tag + "'");
  }
}

void EnvTag::processNode(YAML::Node node) const {
  if (!node.IsScalar()) {
    std::stringstream ss;
    ss << "Node with !env tag is not scalar: '" << node << "'";
    internal::Logger::logWarning(ss.str());
    return;
  }

  std::string varname;
  try {
    varname = node.as<std::string>();
  } catch (YAML::Exception& e) {
    std::stringstream ss;
    ss << "Failed to get envname from; '" << node << "'";
    internal::Logger::logWarning(ss.str());
    return;
  }

  const auto ret = std::getenv(varname.c_str());
  if (!ret) {
    std::stringstream ss;
    ss << "Failed to get envname from; '" << node << "'";
    internal::Logger::logWarning(ss.str());
    return;
  }

  node = std::string(ret);
  node.SetTag("");
}

void resolveTags(YAML::Node node) {
  resolveTag(node);
  switch (node.Type()) {
    case YAML::NodeType::Map:
      for (const auto& child : node) {
        // technically keys can have tags...
        // shouldn't need to recurse
        resolveTag(child.first);
        // dispatch recursion to value
        resolveTags(child.second);
      }
      break;
    case YAML::NodeType::Sequence:
      // dispatch resolution to all children
      for (const auto& child : node) {
        resolveTags(child);
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
