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

#include <exception>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/string_utils.h"

/**
 * @brief This file implements parsing and printing of eigen Matrix types. Pulls in Eigen as a depdenency. By simply
 * including this file, all config_utilities operations on Eigen::Matrix types will be available.
 */

namespace YAML {

template <typename Scalar>
Scalar convertNodeToScalar(const Node& node) {
  return node.as<Scalar>();
}

template <typename Scalar>
void convertScalarToNode(const Scalar& scalar, Node& node) {
  node.push_back(scalar);
}

// Conversion specialization for uint8_t.
template <>
inline uint8_t convertNodeToScalar(const Node& node) {
  return node.as<uint16_t>();
}

template <>
inline void convertScalarToNode(const uint8_t& scalar, Node& node) {
  node.push_back(static_cast<uint16_t>(scalar));
}

template <typename Scalar, int R, int C>
struct convert<Eigen::Matrix<Scalar, R, C>> {
  static Node encode(const Eigen::Matrix<Scalar, R, C>& rhs) {
    Node node;
    // Store the matrix in row-major layout.
    for (int r = 0; r < rhs.rows(); ++r) {
      Node row;
      for (int c = 0; c < rhs.cols(); ++c) {
        convertScalarToNode(rhs(r, c), row);
      }
      node.push_back(row);
    }

    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<Scalar, R, C>& rhs) {
    // Check matrx layout. NOTE(lschmid) Exceptions are caught and printed as errors in the yaml parser.
    if (!node.IsSequence() || node.size() != static_cast<size_t>(rhs.rows())) {
      throw std::runtime_error("Incompatible Matrix dimensions: Requested " + std::to_string(rhs.rows()) + "x" +
                               std::to_string(rhs.cols()) + " but got " + std::to_string(node.size()) + "x" +
                               std::to_string(node[0].size()));
    }
    for (int c = 0; c < rhs.cols(); ++c) {
      if (node[c].size() != static_cast<size_t>(rhs.cols())) {
        throw std::runtime_error("Incompatible Matrix dimensions: Requested " + std::to_string(rhs.rows()) + "x" +
                                 std::to_string(rhs.cols()) + " but got " + std::to_string(node.size()) + "x" +
                                 std::to_string(node[c].size()));
      }
    }

    for (int r = 0; r < rhs.rows(); ++r) {
      for (int c = 0; c < rhs.cols(); ++c) {
        rhs(r, c) = convertNodeToScalar<Scalar>(node[r][c]);
      }
    }

    return true;
  }
};

// Specialization for Rx1 vectors, which will be stored transposed for easier reading and writing.
template <typename Scalar, int R>
struct convert<Eigen::Matrix<Scalar, R, 1>> {
  static Node encode(const Eigen::Matrix<Scalar, R, 1>& rhs) {
    Node node;
    for (int r = 0; r < rhs.rows(); ++r) {
      convertScalarToNode(rhs(r), node);
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<Scalar, R, 1>& rhs) {
    // Check matrx layout. NOTE(lschmid) Exceptions are caught and printed as errors in the yaml parser.
    if (!node.IsSequence() || node.size() != static_cast<size_t>(rhs.rows())) {
      throw std::runtime_error("Incompatible Matrix dimensions: Requested " + std::to_string(rhs.rows()) +
                               "x1 but got " + std::to_string(node.size()) + "x1");
    }

    for (int r = 0; r < rhs.rows(); ++r) {
      rhs(r) = convertNodeToScalar<Scalar>(node[r]);
    }

    return true;
  }
};

}  // namespace YAML
