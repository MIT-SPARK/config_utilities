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
  //! @brief Update original values in matching sequences to the new values. In non-matching cases, the original nodes
  //! prevail.
  UPDATE,
  //! @brief Append new values to the original values in matching sequences. In non-matching cases, the original nodes
  //! prevail.
  APPEND,
  //! @brief Replace original values and sequences with the new values or sequences in matching nodes. In non-matching
  //! cases, the new
  //! nodes overwrite the original nodes.
  REPLACE,
  //! @brief Reset original nodes in conflicting cases to the new nodes. This will stop recursion and result in the
  //! deletion of omitted nodes.
  RESET
};

/**
 * @brief Merges node "from" into "into" with conflicting keys handled by choice of mode
 *
 * Recurses through the YAML tree of "from", adding all non-conflicting nodes to "into". Conflicting nodes (i.e. map
 * keys or shared indices in sequences that already exist in "into") are handled according to the mode selection.
 *
 * - UPDATE will update original values in matching sequences to the new values. In non-matching cases, the original
 * nodes prevail.
 * - APPEND will append new values to the original values in matching sequences. In non-matching cases, the original
 * nodes prevail.
 * - REPLACE will replace original values and sequences with the new values or sequences in matching nodes. In
 * non-matching cases, the new nodes overwrite the original nodes.
 * - RESET will reset original nodes in conflicting cases to the new nodes. This will stop recursion and result in the
 * deletion of omitted nodes.
 *
 * @param into Node to merge into (will be changed).
 * @param from Node to merge from (remains constant).
 * @param mode Mode to use when merging
 */
void mergeYamlNodes(YAML::Node& into, const YAML::Node& from, MergeMode mode = MergeMode::UPDATE);

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

/**
 * @brief Formatting of YAML nodes to strings. Most config types can be neatly represented as low-depth yaml nodes, or
 * should otherwise probably be wrapped in a separate config struct.
 * @param data The data to be formatted.
 * @param reformat_float Whether to try and print floats with default stream precision
 * @returns The formatted string.
 */
std::string yamlToString(const YAML::Node& data, bool reformat_float = false);

}  // namespace config::internal
