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

#include "config_utilities/internal/string_utils.h"

namespace config::internal {

void mergeYamlNodes(YAML::Node& a, const YAML::Node& b) {
  if (!b.IsMap()) {
    // If b is not a map, merge result is b, unless b is null.
    if (b.IsNull() || !b.IsDefined()) {
      return;
    }
    a = YAML::Clone(b);
    return;
  }
  if (!a.IsMap()) {
    // If a is not a map, merge result is b
    a = YAML::Clone(b);
    return;
  }
  if (!b.size()) {
    // If a is a map, and b is an empty map, return a
    return;
  }

  // Merge all entries of b into a.
  for (const auto kv_pair : b) {
    if (kv_pair.first.IsScalar()) {
      const std::string& key = kv_pair.first.Scalar();
      if (a[key]) {
        // Node exists. Merge recursively.
        YAML::Node a_sub = a[key];  // This node is a ref.
        mergeYamlNodes(a_sub, kv_pair.second);
      } else {
        a[key] = YAML::Clone(kv_pair.second);
      }
    }
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
      return true;
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

std::map<std::string, YAML::Node> getNodeMap(const YAML::Node& node) {
  std::map<std::string, YAML::Node> result;
  if (node.IsMap()) {
    for (const auto& kv_pair : node) {
      result.emplace(kv_pair.first.as<std::string>(), kv_pair.second);
    }
  } else if (node.IsSequence()) {
    size_t index = 0;
    for (const auto& sub_node : node) {
      result.emplace(std::to_string(index), sub_node);
      ++index;
    }
  }
  return result;
}

}  // namespace config::internal
