#pragma once

#include <exception>
#include <sstream>
#include <string>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/string_utils.h"

/**
 * @brief This file implements parsing and printing of eigen Matrix types. Pulls in Eigen as a depdenency. By simply
 * including this file, all config_utilities operations on Eigen::Matrix types will be available.
 */

namespace YAML {

// Covnersion specialization for uint8_t.
template <typename Scalar>
Scalar convertNodeToScalar(const Node& node) {
  return node.as<Scalar>();
}
template <>
uint8_t convertNodeToScalar(const Node& node) {
  return node.as<uint16_t>();
}
template <typename Scalar>
void convertScalarToNode(const Scalar& scalar, Node& node) {
  node.push_back(scalar);
}
template <>
void convertScalarToNode(const uint8_t& scalar, Node& node) {
  node.push_back(static_cast<uint16_t>(scalar));
}

template <typename Scalar, int R, int C>
struct convert<Eigen::Matrix<Scalar, R, C>> {
  static Node encode(const Eigen::Matrix<Scalar, R, C>& rhs) {
    Node node;
    // Store the matrix in row-major layout.
    for (int r = 0; r < R; ++r) {
      Node row;
      for (int c = 0; c < C; ++c) {
        convertScalarToNode(rhs(r, c), row);
      }
      node.push_back(row);
    }

    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<Scalar, R, C>& rhs) {
    // Check matrx layout. NOTE(lschmid) Exceptions are caught and printed as errors in the yaml parser.
    if (!node.IsSequence() || node.size() != R) {
      throw std::runtime_error("Incompatible Matrix dimensions: Requested " + std::to_string(R) + "x" +
                               std::to_string(C) + " but got " + std::to_string(node.size()) + "x" +
                               std::to_string(node[0].size()) + ".");
    }
    for (int c = 0; c < C; ++c) {
      if (node[c].size() != C) {
        throw std::runtime_error("Incompatible Matrix dimensions: Requested " + std::to_string(R) + "x" +
                                 std::to_string(C) + " but got " + std::to_string(node.size()) + "x" +
                                 std::to_string(node[c].size()) + ".");
      }
    }

    for (int r = 0; r < R; ++r) {
      for (int c = 0; c < C; ++c) {
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
    for (int r = 0; r < R; ++r) {
      convertScalarToNode(rhs(r), node);
    }
    return node;
  }

  static bool decode(const Node& node, Eigen::Matrix<Scalar, R, 1>& rhs) {
    // Check matrx layout. NOTE(lschmid) Exceptions are caught and printed as errors in the yaml parser.
    if (!node.IsSequence() || node.size() != R) {
      throw std::runtime_error("Incompatible Matrix dimensions: Requested " + std::to_string(R) + "x1 but got " +
                               std::to_string(node.size()) + "x1.");
    }

    for (int r = 0; r < R; ++r) {
      rhs(r) = convertNodeToScalar<Scalar>(node[r]);
    }

    return true;
  }
};

}  // namespace YAML
