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
#include "config_utilities/validation.h"

namespace config::test {

struct UpdateConfig {
  UpdateConfig() = default;
  UpdateConfig(const std::string& s, float f) : s(s), f(f) {}

  std::string s;
  float f = 0.f;
};

void declare_config(UpdateConfig& config) {
  name("UpdateConfig");
  field(config.s, "s");
  field(config.f, "f");
  check(config.f, GE, 0, "f");
}

const std::string yaml_seq = R"(
  s: "a"
  f: 1.0)";

TEST(UpdateConfig, UpdateSuccess) {
  const YAML::Node node = YAML::Load(yaml_seq);
  auto config = fromYaml<UpdateConfig>(node);
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);

  config::updateField(config, "s", "b");
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 1.0f);

  config::updateField(config, "f", 2.0f);
  EXPECT_EQ(config.s, "b");
  EXPECT_EQ(config.f, 2.0f);
}

TEST(UpdateConfig, UpdateFailure) {
  const YAML::Node node = YAML::Load(yaml_seq);
  auto config = fromYaml<UpdateConfig>(node);
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);

  EXPECT_TRUE(isValid(config));
  // EXPECT_FALSE(config::updateField(config, "f", "a"));
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);

  EXPECT_FALSE(config::updateField(config, "f", -1.0f));
  EXPECT_EQ(config.s, "a");
  EXPECT_EQ(config.f, 1.0f);

  EXPECT_TRUE(isValid(config));
}

}  // namespace config::test
