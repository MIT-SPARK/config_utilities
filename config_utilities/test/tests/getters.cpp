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

#include "config_utilities/getters.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"

namespace config::test {

struct GetterStruct {
  int some_number;
  std::string some_string;
};

void declare_config(GetterStruct& config) {
  name("GetterStruct");
  field(config.some_number, "some_number");
  field(config.some_string, "some_string");
}

TEST(ConfigGetters, Getters) {
  const std::string yaml_string = R"yaml(
some_number: 5
some_string: "Hello"
)yaml";
  const auto node = YAML::Load(yaml_string);

  const auto config = fromYaml<GetterStruct>(node);
  EXPECT_EQ(config.some_number, 5);
  EXPECT_EQ(config.some_string, "Hello");

  const auto fields = listFields(config);
  EXPECT_EQ(fields.size(), 2);
  EXPECT_EQ(fields[0], "some_number");
  EXPECT_EQ(fields[1], "some_string");

  const auto number = getField<GetterStruct, int>(config, "some_number");
  EXPECT_TRUE(number.has_value());
  EXPECT_EQ(number.value(), 5);

  const auto string = getField<GetterStruct, std::string>(config, "some_string");
  EXPECT_TRUE(string.has_value());
  EXPECT_EQ(string.value(), "Hello");

  auto logger = TestLogger::create();
  const auto wrong = getField<GetterStruct, int>(config, "some_string");
  EXPECT_FALSE(wrong.has_value());
  EXPECT_EQ(logger->numMessages(), 1);
  EXPECT_EQ(logger->lastMessage(),
            "Field 'some_string' could not be converted to the requested type: yaml-cpp: error at line 1, column 1: "
            "bad conversion");

  const auto wrong2 = getField<GetterStruct, std::string>(config, "non_existent_field");
  EXPECT_FALSE(wrong2.has_value());
  EXPECT_EQ(logger->numMessages(), 2);
  EXPECT_EQ(logger->lastMessage(), "Field 'non_existent_field' not found in config.");
}

}  // namespace config::test
