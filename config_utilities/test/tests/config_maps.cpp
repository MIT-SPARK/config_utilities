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

namespace {
bool use_namespace = false;
}

struct MapConfig {
  MapConfig() = default;
  MapConfig(const std::string& s, float f) : s(s), f(f) {}

  std::string s;
  int f = 0;
};

bool operator==(const MapConfig& lhs, const MapConfig& rhs) { return lhs.s == rhs.s && lhs.f == rhs.f; }

void declare_config(MapConfig& config) {
  name("MapConfig");
  if (use_namespace) {
    enter_namespace("test");
  }
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
  if (use_namespace) {
    enter_namespace("sub_ns");
  }
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

TEST(ConfigMaps, SubConfigSet) {
  const std::string yaml_data = R"(
i: 1
int_map: [{s: "a", f: 1}, {s: "b", f: 2}, {s: "c", f: 3}]
string_map: {x: {f: 4}, y: {f: 5}}
)";
  const auto node = YAML::Load(yaml_data);

  ConfigWithMaps config;
  internal::Visitor::setValues(config, node);

  std::map<size_t, MapConfig> expected_int{{0, {"a", 1}}, {1, {"b", 2}}, {2, {"c", 3}}};
  std::map<std::string, MapConfig> expected_str{{"x", {"", 4}}, {"y", {"", 5}}};

  EXPECT_EQ(config.i, 1);
  EXPECT_EQ(config.int_map, expected_int);
  EXPECT_EQ(config.string_map, expected_str);
}

TEST(ConfigMaps, SubConfigGet) {
  ConfigWithMaps config;
  config.i = 2;
  config.int_map = {{0, {"a", 1}}, {1, {"b", 2}}, {2, {"c", 3}}};
  config.string_map = {{"x", {"", 4}}, {"y", {"", 5}}};

  const auto data = internal::Visitor::getValues(config);
  const auto node = data.data;
  const std::string yaml_data = R"(
i: 2
int_map: {0: {s: "a", f: 1}, 1: {s: "b", f: 2}, 2: {s: "c", f: 3}}
string_map: {x: {s: "", f: 4}, y: {s: "", f: 5}}
)";
  const auto expected = YAML::Load(yaml_data);
  expectEqual(expected, node);
}

TEST(ConfigMaps, SubConfigGetWithNameSpace) {
  ConfigWithMaps config;
  config.i = 2;
  config.int_map = {{0, {"a", 1}}, {1, {"b", 2}}, {2, {"c", 3}}};
  config.string_map = {{"x", {"", 4}}, {"y", {"", 5}}};

  use_namespace = true;
  const auto data = internal::Visitor::getValues(config);
  use_namespace = false;

  const auto node = data.data;
  const std::string yaml_data = R"(
i: 2
sub_ns:
  int_map: {0: {test: {s: "a", f: 1}}, 1: {test: {s: "b", f: 2}}, 2: {test: {s: "c", f: 3}}}
  string_map: {x: {test: {s: "", f: 4}}, y: {test: {s: "", f: 5}}}
  )";
  const auto expected = YAML::Load(yaml_data);
  expectEqual(expected, node);
}

TEST(ConfigMaps, SubConfigCheck) {
  ConfigWithMaps config;
  EXPECT_TRUE(isValid(config));

  config.int_map = {{0, {"a", 1}}, {1, {"b", 2}}, {2, {"c", 3}}};
  EXPECT_TRUE(isValid(config));

  config.int_map.emplace(3, MapConfig{"a", -1});
  config.int_map.emplace(4, MapConfig{"a", -2});
  config.int_map.emplace(5, MapConfig{"a", -3});
  EXPECT_FALSE(isValid(config));

  const auto data = internal::Visitor::getChecks(config);
  int num_checks = 0;
  int num_failed_checks = 0;
  data.performOnAll([&](const auto& d) {
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
  i: 1
  int_map: [{s: "a", f: 1}, {s: "b", f: 2}, {s: "c", f: 3}]
  string_map: {x: {f: 4}, y: {f: 5}}
)";
  const auto node = YAML::Load(yaml_data);

  ConfigWithNestedMaps config;
  internal::Visitor::setValues(config, node);

  std::map<size_t, MapConfig> expected_int{{0, {"a", 1}}, {1, {"b", 2}}, {2, {"c", 3}}};
  std::map<std::string, MapConfig> expected_str{{"x", {"", 4}}, {"y", {"", 5}}};

  EXPECT_EQ(config.nested.i, 1);
  EXPECT_EQ(config.nested.int_map, expected_int);
  EXPECT_EQ(config.nested.string_map, expected_str);
}

TEST(ConfigMaps, PrintMapConfigs) {
  std::map<size_t, MapConfig> configs{{2, {"a", 1}}, {3, {"b", 2}}, {4, {"c", 3}}};
  Settings().print_indent = 20;

  internal::Formatter::setFormatter(std::make_unique<internal::AslFormatter>());

  const std::string formatted = toString(configs);
  const std::string expected = R"(================================== Config Map ==================================
config_map[2] [MapConfig]:
   s:               a
   f:               1
config_map[3] [MapConfig]:
   s:               b
   f:               2
config_map[4] [MapConfig]:
   s:               c
   f:               3
================================================================================)";
  EXPECT_EQ(formatted, expected);
}

}  // namespace config::test
