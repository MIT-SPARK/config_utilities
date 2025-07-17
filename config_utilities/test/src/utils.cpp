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

#include "config_utilities/test/utils.h"

#include <gtest/gtest.h>

namespace config::test {
namespace {

bool expectEqualImpl(const YAML::Node& a, const YAML::Node& b, double epsilon = 0.0) {
  EXPECT_EQ(a.Type(), b.Type());
  if (a.Type() != b.Type()) {
    return false;
  }
  switch (a.Type()) {
    case YAML::NodeType::Scalar:
      if (epsilon > 0.0) {
        // Attempt double conversion and comparison.
        double a_val, b_val;
        if (YAML::convert<double>::decode(a, a_val) && YAML::convert<double>::decode(b, b_val)) {
          EXPECT_NEAR(a_val, b_val, epsilon);
          return std::abs(a_val - b_val) <= epsilon;
        }
      }
      EXPECT_EQ(a.Scalar(), b.Scalar());
      return a.Scalar() == b.Scalar();
    case YAML::NodeType::Sequence:
      EXPECT_EQ(a.size(), b.size());
      if (a.size() != b.size()) {
        return false;
      }
      for (size_t i = 0; i < a.size(); ++i) {
        if (!expectEqualImpl(a[i], b[i], epsilon)) {
          return false;
        }
      }
      return true;
    case YAML::NodeType::Map:
      EXPECT_EQ(a.size(), b.size());
      if (a.size() != b.size()) {
        return false;
      }
      for (const auto& kv_pair : a) {
        const std::string key = kv_pair.first.Scalar();
        if (!b[key]) {
          ADD_FAILURE() << "Key '" << key << "' not found in b.";
          return false;
        }
        if (!expectEqualImpl(kv_pair.second, b[key], epsilon)) {
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

}  // namespace

bool expectEqual(const YAML::Node& a, const YAML::Node& b, double epsilon) {
  const auto equal = expectEqualImpl(a, b, epsilon);
  EXPECT_TRUE(equal) << "---\na:\n---\n" << a << "\n---\nb:\n---\n" << b;
  return equal;
}

void TestLogger::logImpl(const internal::Severity severity, const std::string& message) {
  messages_.emplace_back(severity, message);
}

std::shared_ptr<TestLogger> TestLogger::create() {
  auto logger = std::make_shared<TestLogger>();
  internal::Logger::setLogger(logger);
  return logger;
}

void TestLogger::print() const {
  for (const auto& message : messages_) {
    std::cout << internal::severityToString(message.first) << ": " << message.second << std::endl;
  }
}

}  // namespace config::test
