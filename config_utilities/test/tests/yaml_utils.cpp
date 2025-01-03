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

YAML::Node createData() {
  YAML::Node data;
  data["a"]["b"]["c"] = 1;
  data["a"]["b"]["d"] = "test";
  data["a"]["b"]["e"] = std::vector<float>({1, 2, 3});
  data["a"]["b"]["f"] = std::map<std::string, int>({{"1_str", 1}, {"2_str", 2}});
  data["a"]["g"] = 3;
  return data;
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

}  // namespace config::test
