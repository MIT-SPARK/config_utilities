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

#include <sstream>

#include <gtest/gtest.h>

#include "config_utilities/test/utils.h"

namespace config::test {

using internal::MergeMode;

namespace {

inline YAML::Node createData() {
  YAML::Node data;
  data["a"]["b"]["c"] = 1;
  data["a"]["b"]["d"] = "test";
  data["a"]["b"]["e"] = std::vector<float>({1, 2, 3});
  data["a"]["b"]["f"] = std::map<std::string, int>({{"1_str", 1}, {"2_str", 2}});
  data["a"]["g"] = 3;
  return data;
}

inline YAML::Node doMerge(const YAML::Node& lhs, const YAML::Node& rhs, MergeMode mode = MergeMode::UPDATE) {
  auto result = YAML::Clone(lhs);
  internal::mergeYamlNodes(result, rhs, mode);
  return result;
}

}  // namespace

TEST(YamlUtils, mergeYamlNodes) {
  const auto node_a = YAML::Load(R"""(root: {a: 1, b: 2})""");
  const auto node_b = YAML::Load(R"""(root: {a: 1, c: 3})""");

  const auto result = doMerge(node_a, node_b);
  const auto expected = YAML::Load(R"""(root: {a: 1, b: 2, c: 3})""");
  expectEqual(result, expected);
}

TEST(YamlUtils, mergeYamlNodesInvalidKey) {
  const auto node_a = YAML::Load(R"""(root: {a: 1, b: 2})""");
  const auto node_b = YAML::Load(R"""(? [root, other]: {a: 1, c: 3})""");

  const auto result = doMerge(node_a, node_b);
  const auto expected = YAML::Load(R"""(root: {a: 1, b: 2})""");
  expectEqual(result, expected);
}

TEST(YamlUtils, mergeYamlNodesSequences) {
  // NOTE(nathan) structured to require appending at different levels of recursion
  const auto node_a = YAML::Load(R"""(
root:
  a: 1
  b: [2]
  c:
    foo: [1, 2, 3]
    bar: [{1: 2, 3: 4}, {5, 6}]
other: [1, 2]
)""");
  const auto node_b = YAML::Load(R"""(
root:
  a: 1
  b: [4]
  c:
    bar: [7]
  d: 3.0
other: [3, 4, 5]
)""");

  // Update
  auto result = doMerge(node_a, node_b, MergeMode::UPDATE);
  auto expected = YAML::Load(R"""(
root:
  a: 1
  b: [4]
  c:
    foo: [1, 2, 3]
    bar: [{1: 2, 3: 4}, {5, 6}]
  d: 3.0
other: [3, 4, 5]
)""");
  expectEqual(result, expected);

  // Append
  result = doMerge(node_a, node_b, MergeMode::APPEND);
  expected = YAML::Load(R"""(
root:
  a: 1
  b: [2, 4]
  c:
    foo: [1, 2, 3]
    bar: [{1: 2, 3: 4}, {5, 6}, 7]
  d: 3.0
other: [1, 2, 3, 4, 5]
)""");
  expectEqual(result, expected);

  // Replace
  result = doMerge(node_a, node_b, MergeMode::REPLACE);
  expected = YAML::Load(R"""(
root:
  a: 1
  b: [4]
  c:
    foo: [1, 2, 3]
    bar: [7, {5, 6}]
  d: 3.0
other: [3, 4, 5]
)""");
  expectEqual(result, expected);

  // Reset

  result = doMerge(node_a, node_b, MergeMode::RESET);
  expected = YAML::Load(R"""(
root:
  a: 1
  b: [4]
  c:
    bar: [7]
  d: 3.0
other: [3, 4, 5]
)""");
  expectEqual(result, expected);
}

TEST(YamlUtils, isEqual) {
  {  // different node types are inequal
    const auto node_a = YAML::Load(R"""(5)""");
    const auto node_b = YAML::Load(R"""([5])""");
    EXPECT_TRUE(internal::isEqual(node_a, YAML::Clone(node_a)));
    EXPECT_FALSE(internal::isEqual(node_a, node_b));
  }

  {  // sequences work as expected
    const auto node_a = YAML::Load(R"""([1, 2, 3, 4, 5])""");
    const auto node_b = YAML::Load(R"""([1, 2, 2, 4, 5])""");
    const auto node_c = YAML::Load(R"""([1, 2, 2, 4])""");
    EXPECT_TRUE(internal::isEqual(node_a, YAML::Clone(node_a)));
    EXPECT_FALSE(internal::isEqual(node_a, node_b));
    EXPECT_FALSE(internal::isEqual(node_a, node_c));
    EXPECT_FALSE(internal::isEqual(node_b, node_c));
  }

  {  // maps work as expected
    const auto node_a = YAML::Load(R"""({a: 1, b: 2, c: 3})""");
    const auto node_b = YAML::Load(R"""({a: 1, b: 1, c: 3})""");
    const auto node_c = YAML::Load(R"""({a: 1, d: 1, c: 3})""");
    const auto node_d = YAML::Load(R"""({a: 1, b: 2})""");
    EXPECT_TRUE(internal::isEqual(node_a, YAML::Clone(node_a)));
    EXPECT_FALSE(internal::isEqual(node_a, node_b));
    EXPECT_FALSE(internal::isEqual(node_b, node_c));
    EXPECT_FALSE(internal::isEqual(node_a, node_c));
    EXPECT_FALSE(internal::isEqual(node_b, node_d));
  }

  {  // null nodes are equal
    const auto node_a = YAML::Node();
    const auto node_b = YAML::Node();
    const auto node_c = YAML::Load(R"""({a: 1, d: 1, c: 3})""");
    EXPECT_TRUE(internal::isEqual(node_a, node_b));
    EXPECT_FALSE(internal::isEqual(node_a, node_c));
    EXPECT_FALSE(internal::isEqual(node_b, node_c));
  }
}

TEST(YamlUtils, lookupNamespace) {
  YAML::Node data = createData();

  expectEqual(data, data);

  YAML::Node data_1 = YAML::Clone(data);
  data_1["a"]["b"]["c"] = 2;
  EXPECT_FALSE(internal::isEqual(data, data_1));

  YAML::Node data_2 = internal::lookupNamespace(data, "");
  // NOTE(lschmid): lookupNamespace returns a pointer, so this should be identity.
  EXPECT_TRUE(data == data_2);
  expectEqual(data, data_2);

  YAML::Node b = internal::lookupNamespace(data, "a/b");
  // NOTE(lschmid): lookupNamespace returns a pointer, so this should be identity.
  EXPECT_TRUE(b == data["a"]["b"]);
  expectEqual(b, data["a"]["b"]);

  YAML::Node b2 = internal::lookupNamespace(YAML::Clone(data), "a/b");

  expectEqual(b2, data["a"]["b"]);

  YAML::Node c = internal::lookupNamespace(data, "a/b/c");
  EXPECT_TRUE(c.IsScalar());
  EXPECT_EQ(c.as<int>(), 1);

  YAML::Node invalid = internal::lookupNamespace(data, "a/b/c/d");
  EXPECT_FALSE(invalid.IsDefined());
  EXPECT_FALSE(static_cast<bool>(invalid));

  // Make sure the input node is not modified.
  expectEqual(data, createData());
}

TEST(YamlUtils, moveDownNamespace) {
  YAML::Node data = createData();

  internal::moveDownNamespace(data, "");
  expectEqual(data, createData());

  YAML::Node expected_data;
  expected_data["a"]["b"]["c"] = createData();
  internal::moveDownNamespace(data, "a/b/c");
  expectEqual(data, expected_data);
}

TEST(YamlUtils, mergeWithTags) {
  // NOTE(nathan) structured to require parsing tags at different levels of recursion
  const auto node_a = YAML::Load(R"""(
root:
  b: [2]
  c: [1, 4]
other: [1, 2]
)""");
  const auto node_b = YAML::Load(R"""(
root:
  b: !append [4]
  c: !reset [0]
other: !append [3, 4, 5]
)""");

  {  // check that tags get parsed corrrectly
    auto result = doMerge(node_a, node_b);
    const auto expected = YAML::Load(R"""(
root:
  b: [2, 4]
  c: [0]
other: [1, 2, 3, 4, 5]
)""");
    expectEqual(result, expected);
  }
}

TEST(YamlUtils, mergeWithNestedTags) {
  // NOTE(nathan) structured to require parsing tags at different levels of recursion
  const auto node_a = YAML::Load(R"""(
root:
  children:
    - {a: [2]}
    - {c: [1, 4]}
)""");
  const auto node_b = YAML::Load(R"""(
root:
  children:
    - {a: !append [4]}
    - {c: !reset [0]}
)""");

  {  // check that tags get parsed corrrectly
    auto result = doMerge(node_a, node_b);
    const auto expected = YAML::Load(R"""(
root:
  children:
    - {a: [2, 4]}
    - {c: [0]}
)""");
    expectEqual(result, expected);
  }
}

}  // namespace config::test
