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

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/printing.h"

namespace config::test {

struct FakeVector {
  int x = 0;
  int y = 0;
  int z = 0;
};

void declare_config(FakeVector& vector) {
  name("FakeVector");
  field(vector.x, "x");
  field(vector.y, "y");
  field(vector.z, "z");
}

struct VectorConfig {
  FakeVector a;
  FakeVector b;
};

struct FlatVectorConfig : public VectorConfig {};
struct NamespacedVectorConfig : public VectorConfig {};
struct DoubleNamespacedVectorConfig : public VectorConfig {};

void declare_config(VectorConfig& config) {
  name("VectorConfig");
  field(config.a, "a");
  field(config.b, "b");
}

void declare_config(FlatVectorConfig& config) {
  name("FlatVectorConfig");
  field(config.a, "a", false);
  field(config.b, "b", false);
}

void declare_config(NamespacedVectorConfig& config) {
  name("NamespacedVectorConfig");
  field(config.a, "a");
  NameSpace ns("a");
  field(config.b, "b", false);
}

void declare_config(DoubleNamespacedVectorConfig& config) {
  name("DoubleNamespacedVectorConfig");
  field(config.a, "a");
  NameSpace ns("a");
  field(config.b, "b", true);
}

bool operator==(const FakeVector& lhs, const FakeVector& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator==(const VectorConfig& lhs, const VectorConfig& rhs) { return lhs.a == rhs.a && lhs.b == rhs.b; }

VectorConfig makeExpected(int a_x, int a_y, int a_z, int b_x, int b_y, int b_z) {
  FakeVector a{a_x, a_y, a_z};
  FakeVector b{b_x, b_y, b_z};
  return {a, b};
}

void PrintTo(const VectorConfig& conf, std::ostream* os) { *os << toString(conf); }
void PrintTo(const FlatVectorConfig& conf, std::ostream* os) { *os << toString(conf); }
void PrintTo(const NamespacedVectorConfig& conf, std::ostream* os) { *os << toString(conf); }
void PrintTo(const DoubleNamespacedVectorConfig& conf, std::ostream* os) { *os << toString(conf); }

TEST(Subconfigs, SubconfigNamespacing) {
  const std::string yaml_str = R"(
x: 1
y: 2
z: 3
a:
  x: 4
  y: 5
  z: 6
  b: {x: 7, y: 8, z: 9}
b: {x: -1, y: -2, z: -3}
)";
  const auto node = YAML::Load(yaml_str);

  auto nested_config = fromYaml<VectorConfig>(node);
  auto flat_config = fromYaml<FlatVectorConfig>(node);
  auto namespaced_config = fromYaml<NamespacedVectorConfig>(node);
  auto double_namespaced_config = fromYaml<DoubleNamespacedVectorConfig>(node);
  EXPECT_EQ(makeExpected(4, 5, 6, -1, -2, -3), nested_config);
  EXPECT_EQ(makeExpected(1, 2, 3, 1, 2, 3), flat_config);
  EXPECT_EQ(makeExpected(4, 5, 6, 4, 5, 6), namespaced_config);
  EXPECT_EQ(makeExpected(4, 5, 6, 7, 8, 9), double_namespaced_config);
}

}  // namespace config::test
