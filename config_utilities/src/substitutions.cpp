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

#include <regex>

#include "config_utilities/internal/logger.h"

namespace config {
namespace {

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
      internal::Logger::logError("Unknown substitution '" + tag + "'!");
      context.error();
      return result;
    }

    return parser->process(context, result);
  }

  std::string print() const {
    std::string result;
    result += "{type='" + tag + "', expr='" + expression + "', [";
    auto iter = info_->children.begin();
    while (iter != info_->children.end()) {
      result += iter->print();
      ++iter;
      if (iter != info_->children.end()) {
        result += ", ";
      }
    }

    result += "]}";
    return result;
  }

  SubsNode* parent() const { return info_->parent; }

 private:
  struct Info {
    SubsNode* parent = nullptr;
    std::list<SubsNode> children;
  };

  std::unique_ptr<Info> info_;
};

struct Token {
  enum class Type { Open, Close, Terminator } type;
  std::string contents;

  std::string print() const {
    switch (type) {
      case Type::Open:
        return "<type=Open, contents='" + contents + "'>";
      case Type::Close:
        return "<type=Close, contents='" + contents + "'>";
      case Type::Terminator:
        return "<type=Terminator, contents='" + contents + "'>";
    }

    return "<type=???, contents='" + contents + "'>";
  }
};

inline bool getNextToken(const ParserContext& context, Token& token, std::string& to_search) {
  if (to_search.empty()) {
    return false;
  }

  const std::regex tag_regex(context.prefix + R"""(([\w-]*))""" + context.separator);
  const std::regex suffix_regex(context.suffix);

  std::smatch open;
  std::regex_search(to_search, open, tag_regex);

  std::smatch close;
  std::regex_search(to_search, close, suffix_regex);

  // no more tokens, just return rest of expression
  if (open.empty() && close.empty()) {
    token.type = Token::Type::Terminator;
    token.contents = to_search;
    to_search = "";
    return true;
  }

  bool open_next = open.position() <= close.position();
  const auto* next_match = open_next ? &open : &close;
  if (next_match->prefix().length()) {
    token.type = Token::Type::Terminator;
    token.contents = next_match->prefix();
    to_search = to_search.substr(next_match->position());
    return true;
  }

  if (open_next) {
    token.type = Token::Type::Open;
    token.contents = open.str(1);
    to_search = open.suffix();
    return true;
  }

  token.type = Token::Type::Close;
  to_search = close.suffix();
  return true;
}

inline SubsNode parseSubstitutions(const ParserContext& context, const std::string& original) {
  SubsNode root;
  auto curr_parent = &root;
  std::string to_search = original;

  Token token;
  while (getNextToken(context, token, to_search)) {
    switch (token.type) {
      case Token::Type::Terminator:
        // save any text that isn't a substitution
        curr_parent->addChild().expression = token.contents;
        break;
      case Token::Type::Open: {
        auto& child = curr_parent->addChild();
        child.tag = token.contents;
        curr_parent = &child;
        break;
      }
      case Token::Type::Close: {
        auto new_parent = curr_parent->parent();
        if (new_parent) {
          curr_parent = new_parent;
        } else {
          internal::Logger::logError("Extra closing" + context.suffix + "' found in '" + original + "'");
          context.error();
        }
        break;
      }
    }
  }

  if (curr_parent != &root) {
    internal::Logger::logError("Could not find closing '" + context.suffix + "' for '" + original + "'");
    context.error();
  }

  return root;
}

inline void resolveSubstitution(YAML::Node node, const ParserContext& context) {
  if (!context.allow_substitutions) {
    return;
  }

  if (!node.IsScalar()) {
    return;
  }

  auto to_search = node.as<std::string>();
  auto subs = parseSubstitutions(context, to_search);
  if (!subs || !context) {
    return;
  }

  node = subs.apply(context);
}

void doSubstitutions(YAML::Node node, const ParserContext& context) {
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
        doSubstitutions(child.second, context);
      }
      break;
    case YAML::NodeType::Sequence:
      // dispatch resolution to all children
      for (const auto& child : node) {
        doSubstitutions(child, context);
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

void resolveSubstitutions(YAML::Node node, const ParserContext& context, bool strict) {
  auto to_sub = YAML::Clone(node);
  doSubstitutions(to_sub, context);
  if (context) {
    node = to_sub;
  } else if (strict) {
    throw std::runtime_error("Invalid substitution in node!");
  }
}

}  // namespace config
