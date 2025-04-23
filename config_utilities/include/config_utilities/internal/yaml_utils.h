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
#include <vector>

#include <yaml-cpp/yaml.h>

namespace config::internal {

enum class MergeMode {
  //! @brief Combine the two trees, recursing into matching sequence entries
  UPDATE,
  //! @brief Combine the two trees, appending right sequences into the left
  APPEND,
  //! @brief Combine the two trees, replacing left sequences with the right
  REPLACE
};

/**
 * @brief Merges node b into a with conflicting keys handled by choice of mode
 *
 * Recurses through the YAML "tree" of b, adding all non-conflicting nodes to a. Conflicting nodes (i.e. map keys or
 * shared indices in sequences that already exist in a) are handled according to the mode selection. For `REPLACE`, any
 * conflicting node stops the recursion, and the conflicting node is replaced by the value in b. For 'APPEND', any
 * conflicting sequence node will stop the recursion and cause the entire contents of the node in b to be append to the
 * node in a. For 'UPDATE', any conflicting map or sequence node recursively calls `mergeYamlNodes` with the children of
 * the conflicting nodes as the new roots.
 *
 * @param a Node to merge into ("left" node and will be changed).
 * @param b Node to merge from ("right" node and remains constant).
 * @param mode Mode to use when merging
 */
void mergeYamlNodes(YAML::Node& a, const YAML::Node& b, MergeMode mode = MergeMode::UPDATE);

/**
 * @brief Get a pointer to the final node of the specified namespace if it exists, where each map in the yaml is
 * separated by the separator.
 */
YAML::Node lookupNamespace(const YAML::Node& node, const std::string& name_space, const std::string& separator = "/");

/**
 * @brief Move the node down the specified namespace, where each namespace separated by the separator is represented as
 * a map key.
 */
void moveDownNamespace(YAML::Node& node, const std::string& name_space, const std::string& separator = "/");

/**
 * @brief Check whether two yaml nodes are equal. Note that since since yaml-cpp operator== checks for identity and not
 * equality, scalar values will be compared by string representation.
 */
bool isEqual(const YAML::Node& a, const YAML::Node& b);

/**
 * @brief Convert a yaml node that contains a map or sequence to a list of corresponding nodes.
 * @param node The node to convert.
 * @return The list of nodes. Nodes stored in this struct are references to the original data.
 */
std::vector<YAML::Node> getNodeArray(const YAML::Node& node);

/**
 * @brief Convert a yaml node that contains a map or sequence to a list of corresponding nodes.
 * @param node The node to convert.
 * @return The list of nodes. Nodes stored in this struct are references to the original data.
 */
std::vector<std::pair<YAML::Node, YAML::Node>> getNodeMap(const YAML::Node& node);

}  // namespace config::internal
