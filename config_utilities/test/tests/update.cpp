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

#include "config_utilities/update.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/types/enum.h"
#include "config_utilities/validation.h"

namespace config::test {

enum class TestEnum { kA, kB, kC };

struct UpdateConfig {
  UpdateConfig() = default;
  UpdateConfig(const std::string& s, float f) : s(s), f(f) {}

  std::string s;
  float f = 0.f;
  std::vector<int> v;
  TestEnum e = TestEnum::kA;
};

void declare_config(UpdateConfig& config) {
  name("UpdateConfig");
  field(config.s, "s");
  field(config.f, "f");
  field(config.v, "v");
  enum_field(config.e, "e", {"A", "B", "C"});
  check(config.f, GE, 0, "f");
}

const std::string yaml_seq = R"(
  s: "a"
  f: 1.0
  v: [1, 2, 3]
  e: "A")";

const std::string yaml_update_s = R"(
  s: "b")";

const std::string yaml_update_f = R"(
  f: 2.0)";

const std::string yaml_update_v = R"(
  v: [4, 5, 6])";

const std::string yaml_update_e = R"(
  e: "B")";

TEST(UpdateConfig, UpdateSuccessDirect) {
  const YAML::Node node = YAML::Load(yaml_seq);
  auto config = fromYaml<UpdateConfig>(node);
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_TRUE(updateField(config, "s", "b"));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_TRUE(updateField(config, "f", 2.0f));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 2.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_TRUE(updateField(config, "v", std::vector<int>{4, 5, 6}));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 2.0f);
  EXPECT_EQ(config.v, (std::vector<int>{4, 5, 6}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_TRUE(updateField(config, "e", "B"));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 2.0f);
  EXPECT_EQ(config.v, (std::vector<int>{4, 5, 6}));
  EXPECT_EQ(config.e, TestEnum::kB);

  EXPECT_TRUE(updateFieldEnum(config, "e", TestEnum::kC, {"A", "B", "C"}));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 2.0f);
  EXPECT_EQ(config.v, (std::vector<int>{4, 5, 6}));
  EXPECT_EQ(config.e, TestEnum::kC);
}

TEST(UpdateConfig, UpdateSuccessYAML) {
  const YAML::Node node = YAML::Load(yaml_seq);
  auto config = fromYaml<UpdateConfig>(node);
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  const YAML::Node update_s = YAML::Load(yaml_update_s);
  EXPECT_TRUE(updateFields(config, update_s, "s"));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  const YAML::Node update_f = YAML::Load(yaml_update_f);
  EXPECT_TRUE(updateFields(config, update_f, "f"));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 2.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  const YAML::Node update_v = YAML::Load(yaml_update_v);
  EXPECT_TRUE(updateFields(config, update_v, "v"));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 2.0f);
  EXPECT_EQ(config.v, (std::vector<int>{4, 5, 6}));
  EXPECT_EQ(config.e, TestEnum::kA);

  const YAML::Node update_e = YAML::Load(yaml_update_e);
  EXPECT_TRUE(updateFields(config, update_e, "e"));
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 2.0f);
  EXPECT_EQ(config.v, (std::vector<int>{4, 5, 6}));
  EXPECT_EQ(config.e, TestEnum::kB);
}

TEST(UpdateConfig, UpdateFailure) {
  const YAML::Node node = YAML::Load(yaml_seq);
  auto config = fromYaml<UpdateConfig>(node);
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);

  EXPECT_TRUE(isValid(config));
  EXPECT_FALSE(updateField(config, "f", "a"));
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_FALSE(updateField(config, "f", -1.0f));
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_FALSE(updateField(config, "v", 1));
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_FALSE(updateField(config, "e", -1));
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_FALSE(updateField(config, "e", TestEnum::kA));
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_FALSE(updateFieldEnum(config, "e", TestEnum::kA));
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);
  EXPECT_EQ(config.v, (std::vector<int>{1, 2, 3}));
  EXPECT_EQ(config.e, TestEnum::kA);

  EXPECT_TRUE(isValid(config));
}

}  // namespace config::test
