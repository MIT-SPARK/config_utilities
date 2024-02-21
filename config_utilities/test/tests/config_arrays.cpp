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

struct ArrConfig {
  ArrConfig() = default;
  ArrConfig(const std::string& s, float f) : s(s), f(f) {}

  std::string s;
  float f = 0.f;
};

void declare_config(ArrConfig& config) {
  name("ArrConfig");
  if (use_namespace) {
    enter_namespace("test");
  }
  field(config.s, "s");
  field(config.f, "f");
  check(config.f, GE, 0, "f");
}

struct ConfigWithArrays {
  int i = 0;
  std::vector<ArrConfig> arr;
};

void declare_config(ConfigWithArrays& config) {
  name("ConfigWithArrays");
  field(config.i, "i");
  if (use_namespace) {
    enter_namespace("sub_ns");
  }
  field(config.arr, "arr");
  check(config.i, GE, 0, "i");
}

class ProcessorBase {
 public:
  virtual void process(std::string& s) const = 0;
  virtual ~ProcessorBase() = default;
};

class AddString : public ProcessorBase {
 public:
  struct Config {
    std::string s;
  };

  explicit AddString(const Config& config) : config_(config) {}
  void process(std::string& s) const override { s += config_.s; }

 private:
  const Config config_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<ProcessorBase, AddString, AddString::Config>("AddString");
};

void declare_config(AddString::Config& config) {
  name("AddString");
  field(config.s, "s");
}

struct ConfigWithVirtualArrays {
  std::vector<VirtualConfig<ProcessorBase>> processor_configs;
};

void declare_config(ConfigWithVirtualArrays& config) {
  name("ConfigWithVirtualArrays");
  field(config.processor_configs, "processor_configs");
}

struct ConfigWithNestedArray {
  ConfigWithVirtualArrays nested;
};

void declare_config(ConfigWithNestedArray& config) {
  name("ConfigWithNextedArrays");
  field(config.nested, "nested");
}

TEST(ConfigArrays, FromYamlSeq) {
  const std::string yaml_seq = R"(
- s: "a"
  f: 1.0
- s: "b"
  f: 2.0
- s: "c"
  f: 3.0
)";
  const YAML::Node node = YAML::Load(yaml_seq);

  auto configs = fromYaml<std::vector<ArrConfig>>(node);
  EXPECT_EQ(configs.size(), 3);
  EXPECT_EQ(configs[0].s, "a");
  EXPECT_EQ(configs[0].f, 1.0f);
  EXPECT_EQ(configs[1].s, "b");
  EXPECT_EQ(configs[1].f, 2.0f);
  EXPECT_EQ(configs[2].s, "c");
  EXPECT_EQ(configs[2].f, 3.0f);
}

TEST(ConfigArrays, FromYamlMap) {
  const std::string yaml_map = R"(
x:
  s: "a"
  f: 1.0
y:
  s: "b"
  f: 2.0
z:
  s: "c"
  f: 3.0
)";
  const YAML::Node node = YAML::Load(yaml_map);

  auto configs = fromYaml<std::vector<ArrConfig>>(node);
  EXPECT_EQ(configs.size(), 3);
  EXPECT_EQ(configs[0].s, "a");
  EXPECT_EQ(configs[0].f, 1.0f);
  EXPECT_EQ(configs[1].s, "b");
  EXPECT_EQ(configs[1].f, 2.0f);
  EXPECT_EQ(configs[2].s, "c");
  EXPECT_EQ(configs[2].f, 3.0f);
}

TEST(ConfigArrays, SubConfigSet) {
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

TEST(ConfigArrays, SubConfigGet) {
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

TEST(ConfigArrays, SubConfigGetWithNameSpace) {
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

TEST(ConfigArrays, SubConfigCheck) {
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

TEST(ConfigArrays, VirtualConfig) {
  const std::string yaml_data = R"(
- type: "AddString"
  s: "a"
- type: "AddString"
  s: "b"
- type: "AddString"
  s: "c"
)";
  const YAML::Node node = YAML::Load(yaml_data);

  auto configs = fromYaml<std::vector<VirtualConfig<ProcessorBase>>>(node);
  EXPECT_EQ(configs.size(), 3);
  EXPECT_EQ(configs[0].getType(), "AddString");
  EXPECT_EQ(configs[1].getType(), "AddString");
  EXPECT_EQ(configs[2].getType(), "AddString");
  EXPECT_TRUE(configs[0].isSet());
  EXPECT_TRUE(configs[1].isSet());
  EXPECT_TRUE(configs[2].isSet());

  std::vector<std::unique_ptr<ProcessorBase>> processors;
  for (const auto& config : configs) {
    processors.emplace_back(config.create());
  }
  EXPECT_EQ(processors.size(), 3);

  std::string to_process;
  for (const auto& processor : processors) {
    processor->process(to_process);
  }
  EXPECT_EQ(to_process, "abc");
}

TEST(ConfigArrays, VirtualSubConfig) {
  const std::string yaml_data = R"(
processor_configs:
  - type: "AddString"
    s: "hello"
  - type: "AddString"
    s: " "
  - type: "AddString"
    s: "world"
)";
  const YAML::Node node = YAML::Load(yaml_data);

  auto config = fromYaml<ConfigWithVirtualArrays>(node);
  EXPECT_EQ(config.processor_configs.size(), 3);
  EXPECT_EQ(config.processor_configs[0].getType(), "AddString");
  EXPECT_EQ(config.processor_configs[1].getType(), "AddString");
  EXPECT_EQ(config.processor_configs[2].getType(), "AddString");
  EXPECT_TRUE(config.processor_configs[0].isSet());
  EXPECT_TRUE(config.processor_configs[1].isSet());
  EXPECT_TRUE(config.processor_configs[2].isSet());

  std::vector<std::unique_ptr<ProcessorBase>> processors;
  for (const auto& c : config.processor_configs) {
    processors.emplace_back(c.create());
  }
  EXPECT_EQ(processors.size(), 3);

  std::string to_process;
  for (const auto& processor : processors) {
    processor->process(to_process);
  }
  EXPECT_EQ(to_process, "hello world");
}

TEST(ConfigArrays, NestedVirtualSubConfig) {
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

TEST(ConfigArrays, PrintArrayConfigs) {
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

}  // namespace config::test
