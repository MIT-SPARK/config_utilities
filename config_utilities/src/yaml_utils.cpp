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

#include "config_utilities/internal/yaml_utils.h"

#include <optional>
#include <regex>
#include <sstream>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"

namespace config::internal {
namespace {

inline bool isLeaf(const YAML::Node& a) { return !a.IsMap() && !a.IsSequence(); }

inline std::optional<MergeMode> modeFromTag(const YAML::Node& a) {
  const auto tag = a.Tag();
  if (tag == "!append") {
    return MergeMode::APPEND;
  } else if (tag == "!update") {
    return MergeMode::UPDATE;
  } else if (tag == "!replace") {
    return MergeMode::REPLACE;
  } else if (tag == "!reset") {
    return MergeMode::RESET;
  } else {
    return std::nullopt;
  }
}
inline std::string modeToString(MergeMode mode) {
  switch (mode) {
    case MergeMode::APPEND:
      return "APPEND";
    case MergeMode::UPDATE:
      return "UPDATE";
    case MergeMode::REPLACE:
      return "REPLACE";
    case MergeMode::RESET:
      return "RESET";
    default:
      return "UNKNOWN";
  }
}

inline void mergeLeaves(YAML::Node& a, const YAML::Node& b, MergeMode mode) {
  // If b is invalid, we can't do anything.
  if ((b.IsNull() || !b.IsDefined())) {
    if (mode == MergeMode::RESET) {
      a = YAML::Node();
    }
    return;
  }
  if (!isLeaf(a) && mode != MergeMode::RESET && mode != MergeMode::REPLACE) {
    std::stringstream ss;
    ss << "Cannot merge leaf and non-leaf in mode " << modeToString(mode) << "! Discarding '" << b << "'";
    Logger::logWarning(ss.str());
    return;
  }

  a = YAML::Clone(b);
}

inline void mergeYamlMaps(YAML::Node& a, const YAML::Node& b, MergeMode mode) {
  const auto tag_mode = modeFromTag(b);
  mode = tag_mode.value_or(mode);
  if (mode == MergeMode::RESET) {
    a = YAML::Clone(b);
    if (tag_mode) {
      a.SetTag("");
    }
    return;
  }

  // Both a and b are maps: merge all entries of b into a.
  for (const auto& node : b) {
    if (!node.first.IsScalar()) {
      std::stringstream ss;
      ss << "Non-scalar keys not supported, dropping '" << node.first << "' during merge";
      Logger::logWarning(ss.str());
      continue;
    }

    const auto& key = node.first.Scalar();
    if (a[key]) {
      // Node exists. Merge recursively.
      YAML::Node a_sub = a[key];  // This node is a ref.
      mergeYamlNodes(a_sub, node.second, mode);
    } else {
      // Leaf of a, but b continues: insert b
      a[key] = YAML::Clone(node.second);
    }
  }
}

inline void updateYamlSequence(YAML::Node& a, const YAML::Node& b, MergeMode mode) {
  auto iter_a = a.begin();
  auto iter_b = b.begin();
  while (iter_b != b.end()) {
    if (iter_a != a.end()) {
      auto a_ref = *iter_a;
      mergeYamlNodes(a_ref, *iter_b, mode);
      ++iter_a;
    } else {
      a.push_back(YAML::Clone(*iter_b));
    }
    ++iter_b;
  }
}

inline void mergeYamlSequences(YAML::Node& a, const YAML::Node& b, MergeMode mode) {
  const auto tag_mode = modeFromTag(b);
  mode = tag_mode.value_or(mode);
  switch (mode) {
    case MergeMode::RESET:
      a = YAML::Clone(b);
      if (tag_mode) {
        a.SetTag("");
      }
      break;
    case MergeMode::APPEND:
      for (const auto& child : b) {
        a.push_back(YAML::Clone(child));
      }
      break;
    case MergeMode::REPLACE:
    case MergeMode::UPDATE:
    default:
      updateYamlSequence(a, b, mode);
      break;
  }
}

}  // namespace

void mergeYamlNodes(YAML::Node& a, const YAML::Node& b, MergeMode mode) {
  // If either node is a leaf in the config tree, pass merging behavior to helper function
  if (isLeaf(b) || isLeaf(a)) {
    mergeLeaves(a, b, mode);
    return;
  }

  if (a.IsMap() && b.IsMap()) {
    mergeYamlMaps(a, b, mode);
  } else if (a.IsSequence() && b.IsSequence()) {
    mergeYamlSequences(a, b, mode);
  } else if (mode == MergeMode::RESET || mode == MergeMode::REPLACE) {
    a = YAML::Clone(b);
    if (modeFromTag(b)) {
      a.SetTag("");
    }
  } else {
    std::stringstream ss;
    ss << "Cannot merge map and sequence in mode " << modeToString(mode) << "! Discarding '" << b << "'";
    Logger::logWarning(ss.str());
  }
}

YAML::Node lookupNamespace(const YAML::Node& node, const std::string& name_space, const std::string& separator) {
  YAML::Node current_node(node);
  for (const std::string& ns : splitNamespace(name_space, separator)) {
    if (!current_node.IsMap() || !current_node[ns]) {
      // Full namespace does not exist, make sure to not modify the input node.
      return YAML::Node(YAML::NodeType::Undefined);
    }
    current_node.reset(current_node[ns]);
  }
  return current_node;
}

void moveDownNamespace(YAML::Node& node, const std::string& name_space, const std::string& separator) {
  const std::vector<std::string> namespaces = splitNamespace(name_space, separator);
  for (auto it = namespaces.rbegin(); it != namespaces.rend(); ++it) {
    YAML::Node tmp;   // This is a new node.
    tmp[*it] = node;  // Add the current node as a child.
    node.reset(tmp);
  }
}

bool isEqual(const YAML::Node& a, const YAML::Node& b) {
  if (a.Type() != b.Type()) {
    return false;
  }

  switch (a.Type()) {
    case YAML::NodeType::Scalar:
      return a.Scalar() == b.Scalar();
    case YAML::NodeType::Sequence:
      if (a.size() != b.size()) {
        return false;
      }
      for (size_t i = 0; i < a.size(); ++i) {
        if (!isEqual(a[i], b[i])) {
          return false;
        }
      }
      return true;
    case YAML::NodeType::Map:
      if (a.size() != b.size()) {
        return false;
      }
      for (const auto& kv_pair : a) {
        const std::string key = kv_pair.first.Scalar();
        if (!b[key]) {
          return false;
        }
        if (!isEqual(kv_pair.second, b[key])) {
          return false;
        }
      }
      return true;
    case YAML::NodeType::Null:
    case YAML::NodeType::Undefined:
      return true;
  }

  return false;
}

std::vector<YAML::Node> getNodeArray(const YAML::Node& node) {
  std::vector<YAML::Node> result;
  if (node.IsMap()) {
    for (const auto& kv_pair : node) {
      result.emplace_back(kv_pair.second);
    }
  } else if (node.IsSequence()) {
    for (const auto& sub_node : node) {
      result.emplace_back(sub_node);
    }
  }
  return result;
}

std::vector<std::pair<YAML::Node, YAML::Node>> getNodeMap(const YAML::Node& node) {
  std::vector<std::pair<YAML::Node, YAML::Node>> result;
  if (node.IsMap()) {
    for (const auto& kv_pair : node) {
      result.emplace_back(kv_pair);
    }
  } else if (node.IsSequence()) {
    size_t index = 0;
    for (const auto& sub_node : node) {
      auto& new_pair = result.emplace_back();
      new_pair.first = index;
      new_pair.second = YAML::Clone(sub_node);
      ++index;
    }
  }

  return result;
}

std::string scalarToString(const YAML::Node& data, bool reformat_float) {
  std::stringstream orig;
  orig << data;
  if (!reformat_float) {
    return orig.str();
  }

  const std::regex float_detector("[+-]?[0-9]*[.][0-9]+");
  if (!std::regex_search(orig.str(), float_detector)) {
    return orig.str();  // no reason to reformat if no decimal points
  }

  double value;
  try {
    value = data.as<double>();
  } catch (const std::exception&) {
    return orig.str();  // value is some sort of string that can't be parsed as a float
  }

  // this should have default ostream precision for formatting float
  std::stringstream ss;
  ss << value;
  return ss.str();
}

std::string yamlToString(const YAML::Node& data, bool reformat_float) {
  switch (data.Type()) {
    case YAML::NodeType::Scalar: {
      // scalars require special handling for float precision
      return scalarToString(data, reformat_float);
    }
    case YAML::NodeType::Sequence: {
      std::string result = "[";
      for (size_t i = 0; i < data.size(); ++i) {
        result += yamlToString(data[i], reformat_float);
        if (i < data.size() - 1) {
          result += ", ";
        }
      }
      result += "]";
      return result;
    }
    case YAML::NodeType::Map: {
      std::string result = "{";
      bool has_data = false;
      for (const auto& kv_pair : data) {
        has_data = true;
        result +=
            yamlToString(kv_pair.first, reformat_float) + ": " + yamlToString(kv_pair.second, reformat_float) + ", ";
      }
      if (has_data) {
        result = result.substr(0, result.length() - 2);
      }
      result += "}";
      return result;
    }
    default:
      return kInvalidField;
  }
}

}  // namespace config::internal
