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
#include "config_utilities/factory.h"
#include "config_utilities/formatting/asl.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/printing.h"
#include "config_utilities/settings.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/validation.h"
#include "config_utilities/virtual_config.h"

namespace config::test {

struct MapConfig {
  MapConfig() = default;
  MapConfig(const std::string& s, float f) : s(s), f(f) {}

  std::string s;
  int f = 0;
};

bool operator==(const MapConfig& lhs, const MapConfig& rhs) { return lhs.s == rhs.s && lhs.f == rhs.f; }

void declare_config(MapConfig& config) {
  name("MapConfig");
  field(config.s, "s");
  field(config.f, "f");
  check(config.f, GE, 0, "f");
}

void PrintTo(const MapConfig& conf, std::ostream* os) { *os << toString(conf); }

struct ConfigWithMaps {
  int i = 0;
  std::map<std::string, MapConfig> string_map;
  std::map<size_t, MapConfig> int_map;
};

void declare_config(ConfigWithMaps& config) {
  name("ConfigWithMaps");
  field(config.i, "i");
  field(config.string_map, "string_map");
  field(config.int_map, "int_map");
  check(config.i, GE, 0, "i");
}

struct ConfigWithNestedMaps {
  ConfigWithMaps nested;
};

void declare_config(ConfigWithNestedMaps& config) {
  name("ConfigWithNestedMaps");
  field(config.nested, "nested");
}

TEST(ConfigMaps, FromYamlMap) {
  const std::string yaml_map = R"(
x:
  s: "a"
  f: 1
y:
  s: "b"
  f: 2
z:
  s: "c"
  f: 3
)";
  const auto node = YAML::Load(yaml_map);

  const auto configs = fromYaml<std::map<std::string, MapConfig>>(node);
  std::map<std::string, MapConfig> expected{{"x", {"a", 1}}, {"y", {"b", 2}}, {"z", {"c", 3}}};
  EXPECT_EQ(configs, expected);
}

TEST(ConfigMaps, FromYamlSeq) {
  const std::string yaml_seq = R"(
- s: "a"
  f: 1
- s: "b"
  f: 2
- s: "c"
  f: 3
)";
  const auto node = YAML::Load(yaml_seq);

  const auto configs = fromYaml<std::map<int, MapConfig>>(node);
  std::map<int, MapConfig> expected{{0, {"a", 1}}, {1, {"b", 2}}, {2, {"c", 3}}};
  EXPECT_EQ(configs, expected);
}

/*
TEST(ConfigMaps, SubConfigSet) {
  const std::string yaml_data = R"(
i: 1
arr:
  - s: "a"
    f: 1.0
  - s: "b"
    f: 2.0
  - s: "c"
    f: 3.0
)";
  YAML::Node node = YAML::Load(yaml_data);

  ConfigWithArrays config;
  internal::Visitor::setValues(config, node);

  EXPECT_EQ(config.i, 1);
  EXPECT_EQ(config.arr.size(), 3);
  EXPECT_EQ(config.arr[0].s, "a");
  EXPECT_EQ(config.arr[0].f, 1.0f);
  EXPECT_EQ(config.arr[1].s, "b");
  EXPECT_EQ(config.arr[1].f, 2.0f);
  EXPECT_EQ(config.arr[2].s, "c");
  EXPECT_EQ(config.arr[2].f, 3.0f);
}

TEST(ConfigMaps, SubConfigGet) {
  ConfigWithArrays config;
  config.arr.emplace_back("a", 1.0f);
  config.arr.emplace_back("b", 2.0f);
  config.arr.emplace_back("c", 3.0f);

  const internal::MetaData data = internal::Visitor::getValues(config);
  const YAML::Node node = data.data;
  const std::string yaml_data = R"(
i: 0
arr:
  - s: "a"
    f: 1
  - s: "b"
    f: 2
  - s: "c"
    f: 3
  )";
  const YAML::Node expected = YAML::Load(yaml_data);

  expectEqual(expected, node);
}

TEST(ConfigMaps, SubConfigGetWithNameSpace) {
  ConfigWithArrays config;
  config.arr.emplace_back("a", 1.0f);
  config.arr.emplace_back("b", 2.0f);
  config.arr.emplace_back("c", 3.0f);

  use_namespace = true;
  const internal::MetaData data = internal::Visitor::getValues(config);
  use_namespace = false;

  const YAML::Node node = data.data;
  const std::string yaml_data = R"(
i: 0
sub_ns:
  arr:
    - test:
        s: "a"
        f: 1
    - test:
        s: "b"
        f: 2
    - test:
        s: "c"
        f: 3
  )";
  const YAML::Node expected = YAML::Load(yaml_data);

  expectEqual(expected, node);
}

TEST(ConfigMaps, SubConfigCheck) {
  ConfigWithArrays config;
  EXPECT_TRUE(isValid(config));

  config.arr.emplace_back("a", 1.0f);
  config.arr.emplace_back("b", 2.0f);
  config.arr.emplace_back("c", 3.0f);
  EXPECT_TRUE(isValid(config));

  config.arr.emplace_back("a", -1.0f);
  config.arr.emplace_back("b", -2.0f);
  config.arr.emplace_back("c", -3.0f);
  EXPECT_FALSE(isValid(config));

  internal::MetaData data = internal::Visitor::getChecks(config);
  int num_checks = 0;
  int num_failed_checks = 0;
  data.performOnAll([&](const internal::MetaData& d) {
    for (const auto& check : d.checks) {
      num_checks++;
      if (!check->valid()) {
        num_failed_checks++;
      }
    }
  });
  EXPECT_EQ(num_checks, 7);
  EXPECT_EQ(num_failed_checks, 3);
}

TEST(ConfigMaps, NestedSubConfig) {
  const std::string yaml_data = R"(
nested:
  processor_configs:
    - type: "AddString"
      s: "hello"
    - type: "AddString"
      s: " "
    - type: "AddString"
      s: "world"
)";
  const YAML::Node node = YAML::Load(yaml_data);

  auto config = fromYaml<ConfigWithNestedArray>(node);
  ASSERT_EQ(config.nested.processor_configs.size(), 3);
  EXPECT_EQ(config.nested.processor_configs[0].getType(), "AddString");
  EXPECT_EQ(config.nested.processor_configs[1].getType(), "AddString");
  EXPECT_EQ(config.nested.processor_configs[2].getType(), "AddString");
  EXPECT_TRUE(config.nested.processor_configs[0].isSet());
  EXPECT_TRUE(config.nested.processor_configs[1].isSet());
  EXPECT_TRUE(config.nested.processor_configs[2].isSet());

  std::vector<std::unique_ptr<ProcessorBase>> processors;
  for (const auto& c : config.nested.processor_configs) {
    processors.emplace_back(c.create());
  }
  ASSERT_EQ(processors.size(), 3);

  std::string to_process;
  for (const auto& processor : processors) {
    processor->process(to_process);
  }
  EXPECT_EQ(to_process, "hello world");
}

TEST(ConfigMaps, PrintMapConfigs) {
  std::vector<ArrConfig> configs;
  configs.emplace_back("a", 1.0f);
  configs.emplace_back("b", 2.0f);
  configs.emplace_back("c", 3.0f);
  Settings().print_indent = 20;

  internal::Formatter::setFormatter(std::make_unique<internal::AslFormatter>());

  const std::string formatted = toString(configs);
  const std::string expected = R"(================================= Config Array =================================
config_array[0] [ArrConfig]:
   s:               a
   f:               1
config_array[1] [ArrConfig]:
   s:               b
   f:               2
config_array[2] [ArrConfig]:
   s:               c
   f:               3
================================================================================)";
  EXPECT_EQ(formatted, expected);
}
*/

}  // namespace config::test
