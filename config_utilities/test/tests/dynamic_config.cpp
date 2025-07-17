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

#include "config_utilities/dynamic_config.h"

#include <gtest/gtest.h>

#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"

namespace config::test {

DefaultConfig modified_config() {
  DefaultConfig config;
  config.i = 2;
  config.f = 3.2f;
  config.vec = {7, 8, 9};
  config.sub_config.i = 3;
  return config;
}

TEST(DynamicConfig, CheckRegistered) {
  DynamicConfigServer server;

  // No dynamic configs registered.
  EXPECT_EQ(server.registeredConfigs().empty(), true);

  // Register a dynamic config.
  {
    auto dyn1 = DynamicConfig<DefaultConfig>("dynamic_config_1");
    auto dyn2 = DynamicConfig<DefaultConfig>("dynamic_config_2", modified_config());
    const auto registered = server.registeredConfigs();
    EXPECT_EQ(registered.size(), 2);
    EXPECT_TRUE(std::find(registered.begin(), registered.end(), "dynamic_config_1") != registered.end());
    EXPECT_TRUE(std::find(registered.begin(), registered.end(), "dynamic_config_2") != registered.end());

    // Check names unique.
    auto logger = TestLogger::create();
    auto dyn3 = DynamicConfig<DefaultConfig>("dynamic_config_1");
    EXPECT_EQ(logger->numMessages(), 1);
    EXPECT_EQ(logger->lastMessage(), "Cannot register dynamic config: key 'dynamic_config_1' already exists.");
  }

  // Dynamic configs should deregister automatically.
  EXPECT_EQ(server.registeredConfigs().empty(), true);
}

TEST(DynamicConfig, SetGet) {
  DynamicConfig<DefaultConfig> dyn("dyn");
  DynamicConfigServer server;

  // Get values.
  auto values = server.get("dyn");
  EXPECT_TRUE(expectEqual(values, DefaultConfig::defaultValues()));

  // Set values.
  std::string yaml_str = R"(
    i: 7
    f: 7.7
    vec: [7, 7, 7]
    sub_ns:
        i: 7
  )";
  auto yaml = YAML::Load(yaml_str);
  server.set("dyn", yaml);

  // Check actual values.
  auto config = dyn.get();
  EXPECT_EQ(config.i, 7);
  EXPECT_EQ(config.f, 7.7f);
  EXPECT_EQ(config.vec, std::vector<int>({7, 7, 7}));
  EXPECT_EQ(config.sub_config.i, 7);

  // Check serialized values.
  values = server.get("dyn");
  EXPECT_EQ(values["i"].as<int>(), 7);
  EXPECT_EQ(values["f"].as<float>(), 7.7f);
  EXPECT_EQ(values["vec"].as<std::vector<int>>(), std::vector<int>({7, 7, 7}));
  EXPECT_EQ(values["sub_ns"]["i"].as<int>(), 7);
  EXPECT_EQ(values["u8"].as<int>(), 4);  // Default value.

  // Check invalid key.
  values = server.get("invalid");
  EXPECT_TRUE(values.IsNull());
  server.set("invalid", DefaultConfig::defaultValues());
  EXPECT_EQ(dyn.get().i, 7);
}

TEST(DynamicConfig, getInfo) {
  DynamicConfig<DefaultConfig> dyn("dyn");
  DynamicConfigServer server;

  // Get values.
  auto info = server.getInfo("dyn");

  const std::string expected_info = R"(
type: config
name: DefaultConfig
fields:
  - type: field
    name: i
    unit: m
    value: 1
    default: 1
    input_info:
      type: int32
      min: 0
      lower_exclusive: true
  - type: field
    name: f
    unit: s
    value: 2.0999999
    default: 2.0999999
    input_info:
      type: float32
      min: 0
  - type: field
    name: d
    unit: m/s
    value: 3.2000000000000002
    default: 3.2000000000000002
    input_info:
      type: float64
      min: 0
      max: 4
      upper_exclusive: true
  - type: field
    name: b
    value: true
    default: true
    input_info:
      type: bool
  - type: field
    name: u8
    value: 4
    default: 4
    input_info:
      type: uint8
      max: 5
  - type: field
    name: s
    value: test string
    default: test string
    input_info:
      type: string
  - type: field
    name: vec
    unit: frames
    value:
      - 1
      - 2
      - 3
    default:
      - 1
      - 2
      - 3
    input_info:
      type: yaml
  - type: field
    name: map
    value:
      a: 1
      b: 2
      c: 3
    default:
      a: 1
      b: 2
      c: 3
    input_info:
      type: yaml
  - type: field
    name: set
    value:
      - 1.10000002
      - 2.20000005
      - 3.29999995
    default:
      - 1.10000002
      - 2.20000005
      - 3.29999995
    input_info:
      type: yaml
  - type: field
    name: mat
    value:
      -
        - 1
        - 0
        - 0
      -
        - 0
        - 1
        - 0
      -
        - 0
        - 0
        - 1
    default:
      -
        - 1
        - 0
        - 0
      -
        - 0
        - 1
        - 0
      -
        - 0
        - 0
        - 1
    input_info:
      type: yaml
  - type: field
    name: my_enum
    value: A
    default: A
    input_info:
      type: options
      options:
        - A
        - B
        - C
  - type: field
    name: my_strange_enum
    value: X
    default: X
    input_info:
      type: options
      options:
        - Z
        - X
        - Y
  - type: config
    name: SubConfig
    field_name: sub_config
    fields:
      - type: field
        name: i
        value: 1
        default: 1
        input_info:
          type: int32
          min: 0
          lower_exclusive: true
      - type: config
        name: SubSubConfig
        field_name: sub_sub_config
        fields:
          - type: field
            name: i
            value: 1
            default: 1
            input_info:
              type: int32
              min: 0
              lower_exclusive: true
  - type: config
    name: SubSubConfig
    field_name: sub_sub_config
    fields:
      - type: field
        name: i
        value: 1
        default: 1
        input_info:
          type: int32
          min: 0
          lower_exclusive: true
)";
  EXPECT_TRUE(expectEqual(info, YAML::Load(expected_info), 1e-6));
}

TEST(DynamicConfig, Hooks) {
  auto server = std::make_unique<DynamicConfigServer>();
  std::string logs;

  // Register hooks.
  DynamicConfigServer::Hooks hooks;
  hooks.onRegister = [&logs](const std::string& key) { logs += "register " + key + "; "; };
  hooks.onDeregister = [&logs](const std::string& key) { logs += "deregister " + key + "; "; };
  hooks.onUpdate = [&logs](const std::string& key, const YAML::Node&) { logs += "update " + key + "; "; };
  server->setHooks(hooks);

  // Register a dynamic config.
  auto a = std::make_unique<DynamicConfig<DefaultConfig>>("A");
  auto b = std::make_unique<DynamicConfig<DefaultConfig>>("B");
  DefaultConfig config;
  a->set(config);  // Should be identical, so not trigger update.
  config.i = 123;
  b->set(config);  // Should trigger update.
  b.reset();
  a.reset();
  EXPECT_EQ(logs, "register A; register B; update B; deregister B; deregister A; ");

  // Update hooks.
  hooks.onRegister = [&logs](const std::string& key) { logs += "register " + key + " again; "; };
  hooks.onDeregister = nullptr;
  server->setHooks(hooks);
  logs.clear();

  // Register a dynamic config.
  auto c = std::make_unique<DynamicConfig<DefaultConfig>>("C");
  c.reset();
  EXPECT_EQ(logs, "register C again; ");

  // Deregister hooks.
  server.reset();
  logs.clear();

  // Register a dynamic config.
  auto d = std::make_unique<DynamicConfig<DefaultConfig>>("D");
  d.reset();
  EXPECT_EQ(logs, "");
}

TEST(DynamicConfig, Move) {
  DynamicConfigServer server;

  // Register a dynamic config.
  DefaultConfig config;
  config.i = 123;
  auto dyn = DynamicConfig<DefaultConfig>("dyn", config);
  EXPECT_EQ(server.registeredConfigs().size(), 1);
  EXPECT_EQ(server.registeredConfigs()[0], "dyn");

  // Move constructor.
  DynamicConfig<DefaultConfig> dyn2(std::move(dyn));
  EXPECT_EQ(server.registeredConfigs().size(), 1);
  EXPECT_EQ(server.registeredConfigs()[0], "dyn");
  EXPECT_EQ(dyn2.get().i, 123);

  // Get/set.
  YAML::Node update = YAML::Load("i: 456");
  server.set("dyn", update);
  EXPECT_EQ(dyn2.get().i, 456);
  config.i = 456;
  config.f = 2.3f;
  dyn2.set(config);
  auto values = server.get("dyn");
  EXPECT_EQ(values["i"].as<int>(), 456);
  EXPECT_EQ(values["f"].as<float>(), 2.3f);

  // Move assignment.
  DynamicConfig<DefaultConfig> dyn3("dyn3");
  EXPECT_EQ(server.registeredConfigs().size(), 2);
  EXPECT_EQ(server.registeredConfigs()[0], "dyn3");
  EXPECT_EQ(server.registeredConfigs()[1], "dyn");

  dyn3 = std::move(dyn2);
  EXPECT_EQ(server.registeredConfigs().size(), 1);
  EXPECT_EQ(server.registeredConfigs()[0], "dyn");
  EXPECT_EQ(dyn3.get().i, 456);
  EXPECT_EQ(dyn3.get().f, 2.3f);

  // Get/set.
  update = YAML::Load("i: 789");
  server.set("dyn", update);
  EXPECT_EQ(dyn3.get().i, 789);
  config.i = 789;
  config.f = 4.5f;
  dyn3.set(config);
  values = server.get("dyn");
  EXPECT_EQ(values["i"].as<int>(), 789);
  EXPECT_EQ(values["f"].as<float>(), 4.5f);
}

}  // namespace config::test
