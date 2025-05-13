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
#include <iostream>
#include <regex>
#include <sstream>

#include "config_utilities/internal/logger.h"

namespace config {
namespace {

static const auto env_reg = RegisteredSubstitutions::Registration<EnvSubstitution>();

class SubsNode {
 public:
  std::string tag;
  std::string expression;

  SubsNode() : info_(new Info()) {}

  SubsNode& addChild() {
    auto& child = info_->children.emplace_back();
    child.info_->parent = this;
    return child;
  }

  operator bool() const {
    const auto root = tag.empty() && expression.empty();
    return !(root && info_->children.empty());
  }

  std::string apply(const ParserContext& context) const {
    std::string result = expression;
    for (const auto& child : info_->children) {
      result += child.apply(context);
    }

    if (tag.empty()) {
      return result;
    }

    auto parser = RegisteredSubstitutions::getEntry(tag);
    if (!parser) {
      internal::Logger::logWarning("Unknown substitution '" + tag + "'!");
      return result;
    }

    return parser->process(context, result);
  }

  std::string print(int level = 0) const {
    std::stringstream ss;
    if (level) {
      ss << std::string(2 * level, ' ');
    }

    ss << "- tag: '" << tag << "', expr: '" << expression << "'\n";
    for (const auto& child : info_->children) {
      ss << child.print(level + 1);
    }

    return ss.str();
  }

 private:
  struct Info {
    const SubsNode* parent = nullptr;
    std::list<SubsNode> children;
  };

  std::unique_ptr<Info> info_;
};

inline SubsNode parseSubstitutions(const ParserContext& context, YAML::Node node) {
  auto to_search = node.as<std::string>();
  std::cout << "Parsing '" << to_search << "'" << std::endl;
  const std::regex tag_regex(context.prefix + R"""(([\w-]*))""" + context.separator);
  const std::regex suffix_regex(R"""(\s*?(.*?))""" + context.suffix);

  std::smatch test;
  if (std::regex_search(to_search, test, tag_regex)) {
    std::cout << "test match: '" << test.str() << "'" << std::endl;
  } else {
    std::cout << "no match!" << std::endl;
  }

  SubsNode root;
  auto curr_parent = &root;
  for (std::smatch m; std::regex_search(to_search, m, tag_regex);) {
    std::cout << "curr tag: '" << m.str(1) << "', total: '" << m.str() << "'" << std::endl;
    to_search = m.suffix();
    auto& curr_sub = curr_parent->addChild();
    curr_sub.tag = m.str(1);

    // search for the next open and close for substitutions if they exist
    std::smatch next_open;
    std::regex_search(to_search, next_open, tag_regex);
    std::smatch next_close;
    std::regex_search(to_search, next_close, suffix_regex);
    if (next_open.empty() && next_close.empty()) {
      internal::Logger::logError("Invalid substitution! Could not find closing '" + context.suffix + "' for '" +
                                 m.str() + "'");
      return SubsNode();
    }

    std::cout << "curr close: '" << next_close.str(1) << "', total: '" << next_close.str() << "'" << std::endl;
    curr_sub.expression = next_close.str(1);
    // no more substitutions, we can continue parsing
    if (next_open.empty()) {
      break;
    }

    // compare positions to determine whether the next closing token occurs
    // before or after the next opening token, which determines whether we
    // have a new sibling or a new child
    const auto has_child = next_open.position() < next_close.position();
    if (!has_child) {
      continue;
    }

    curr_parent = &curr_sub;
  }

  return root;
}

inline void resolveSubstitution(YAML::Node node, const ParserContext& context) {
  if (!node.IsScalar()) {
    return;
  }

  const auto subs = parseSubstitutions(context, node);
  std::cout << "parsed:\n" << subs.print() << std::endl;
  if (!subs) {
    return;
  }

  node = subs.apply(context);
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

std::string EnvSubstitution::process(const ParserContext&, const std::string& contents) const {
  const auto ret = std::getenv(contents.c_str());
  if (!ret) {
    std::stringstream ss;
    ss << "Failed to get envname from '" << contents << "'";
    internal::Logger::logWarning(ss.str());
    return contents;
  }

  return std::string(ret);
}

void resolveSubstitutions(YAML::Node node, const ParserContext& context) {
  const auto tag = node.Tag();
  if (tag == "!append" || tag == "!update" || tag == "!replace") {
    node.SetTag("");
  }

  switch (node.Type()) {
    case YAML::NodeType::Map:
      for (const auto& child : node) {
        // technically keys can have tags...
        // shouldn't need to recurse
        resolveSubstitution(child.first, context);
        // dispatch recursion to value
        resolveSubstitutions(child.second, context);
      }
      break;
    case YAML::NodeType::Sequence:
      // dispatch resolution to all children
      for (const auto& child : node) {
        resolveSubstitutions(child, context);
      }
      break;
    case YAML::NodeType::Scalar:
      resolveSubstitution(node, context);
      break;
    case YAML::NodeType::Undefined:
    case YAML::NodeType::Null:
    default:
      return;
  }
}

}  // namespace config
