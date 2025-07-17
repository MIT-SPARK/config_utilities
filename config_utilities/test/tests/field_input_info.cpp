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

#include <config_utilities/config.h>
#include <config_utilities/internal/visitor.h>
#include <config_utilities/virtual_config.h>
#include <gtest/gtest.h>

#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"

namespace config::test {
namespace {

struct Base {};

struct BaseImpl : Base {
  struct Config {
    float a = 1.0;
  } const config;
  explicit BaseImpl(const Config& config) : config(config) {}
};

struct Parent {};

struct ParentImpl : Parent {
  struct Config {
    config::VirtualConfig<Base> base;
    std::string other = "";
  } const config;
  explicit ParentImpl(const Config& config) : config(config) {}
};

struct MixedConfig {
  VirtualConfig<Base> base;
  VirtualConfig<Parent> parent;
  std::string foo = "bar";
};

void declare_config(BaseImpl::Config& config) {
  name("Impl::Config");
  field(config.a, "a");
}

void declare_config(ParentImpl::Config& config) {
  name("ParentImpl::Config");
  field(config.base, "base");
  field(config.other, "other");
}

void declare_config(MixedConfig& config) {
  name("MixedConfig");
  field(config.base, "base");
  config.parent.setOptional();
  field(config.parent, "parent");
  field(config.foo, "foo");
}

}  // namespace

TEST(FieldInputInfo, GetInfo) {
  DefaultConfig config;
  const internal::MetaData data = internal::Visitor::getInfo(config);
  auto info = data.serializeFieldInfos();
  const std::string expected = R"(
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
    value: 2.1
    default: 2.1
    input_info:
      type: float32
      min: 0
  - type: field
    name: d
    unit: m/s
    value: 3.2
    default: 3.2
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
      - 1.1
      - 2.2
      - 3.3
    default:
      - 1.1
      - 2.2
      - 3.3
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
  // Epect near equal for floating point values.
  expectEqual(info, YAML::Load(expected), 1e-6);
}

TEST(FieldInputInfo, GetVirtualInfo) {
  const auto reg_a = RegistrationGuard<Base, BaseImpl, BaseImpl::Config>("BaseImpl");

  {  // uninitialized config
    config::VirtualConfig<Base> test;
    const auto data = internal::Visitor::getInfo(test);
    auto info = data.serializeFieldInfos();
    const std::string expected = R"(
type: config
name: Uninitialized Virtual Config
available_types: [BaseImpl]
fields: []
)";

    expectEqual(info, YAML::Load(expected), 1e-6);
  }

  {  // init config
    config::VirtualConfig<Base> test{BaseImpl::Config{}};
    const auto data = internal::Visitor::getInfo(test);
    auto info = data.serializeFieldInfos();
    const std::string expected = R"(
type: config
name: BaseImpl
available_types: [BaseImpl]
fields:
  - {type: field, name: a, value: 1, default: 1, input_info: {type: float32}}
)";

    expectEqual(info, YAML::Load(expected), 1e-6);
  }
}

TEST(FieldInputInfo, GetNestedVirtualInfo) {
  const auto reg_a = RegistrationGuard<Base, BaseImpl, BaseImpl::Config>("BaseImpl");
  const auto reg_b = RegistrationGuard<Parent, ParentImpl, ParentImpl::Config>("ParentImpl");

  {  // init parent config
    config::VirtualConfig<Parent> test{ParentImpl::Config{}};
    const auto data = internal::Visitor::getInfo(test);
    auto info = data.serializeFieldInfos();
    const std::string expected = R"(
type: config
name: ParentImpl
available_types: [ParentImpl]
fields:
  - {type: field, name: other, value: '', default: '', input_info: {type: string}}
  - {type: config, name: Uninitialized Virtual Config, field_name: base, available_types: [BaseImpl], fields: []}
)";

    expectEqual(info, YAML::Load(expected), 1e-6);
  }

  {  // init parent config
    config::VirtualConfig<Parent> test{ParentImpl::Config{config::VirtualConfig<Base>{BaseImpl::Config{}}, "hello"}};
    const auto data = internal::Visitor::getInfo(test);
    auto info = data.serializeFieldInfos();
    const std::string expected = R"(
type: config
name: ParentImpl
available_types: [ParentImpl]
fields:
  - {type: field, name: other, value: hello, default: '', input_info: {type: string}}
  - type: config
    name: BaseImpl
    field_name: base
    available_types: [BaseImpl]
    fields:
      - {type: field, name: a, value: 1, default: 1, input_info: {type: float32}}
)";

    expectEqual(info, YAML::Load(expected), 1e-6);
  }
}

TEST(FieldInputInfo, GetVirtualSubconfig) {
  const auto reg_a = RegistrationGuard<Base, BaseImpl, BaseImpl::Config>("BaseImpl");
  const auto reg_b = RegistrationGuard<Parent, ParentImpl, ParentImpl::Config>("ParentImpl");

  {  // init parent config
    config::VirtualConfig<Parent> test{ParentImpl::Config{}};
    const auto data = internal::Visitor::getInfo(test);
    auto info = data.serializeFieldInfos();
    const std::string expected = R"(
type: config
name: ParentImpl
available_types: [ParentImpl]
fields:
  - {type: field, name: other, value: '', default: '', input_info: {type: string}}
  - {type: config, name: Uninitialized Virtual Config, field_name: base, available_types: [BaseImpl], fields: []}
)";

    expectEqual(info, YAML::Load(expected), 1e-6);
  }

  {  // init parent config
    config::VirtualConfig<Parent> test{ParentImpl::Config{config::VirtualConfig<Base>{BaseImpl::Config{}}, "hello"}};
    const auto data = internal::Visitor::getInfo(test);
    auto info = data.serializeFieldInfos();
    const std::string expected = R"(
type: config
name: ParentImpl
available_types: [ParentImpl]
fields:
  - {type: field, name: other, value: hello, default: '', input_info: {type: string}}
  - type: config
    name: BaseImpl
    field_name: base
    available_types: [BaseImpl]
    fields:
      - {type: field, name: a, value: 1, default: 1, input_info: {type: float32}}
)";

    expectEqual(info, YAML::Load(expected), 1e-6);
  }
}

TEST(FieldInputInfo, GetConfigWithVirtuals) {
  const auto reg_a = RegistrationGuard<Base, BaseImpl, BaseImpl::Config>("BaseImpl");
  const auto reg_b = RegistrationGuard<Parent, ParentImpl, ParentImpl::Config>("ParentImpl");

  MixedConfig test{VirtualConfig<Base>{BaseImpl::Config{}},
                   VirtualConfig<Parent>{ParentImpl::Config{VirtualConfig<Base>{BaseImpl::Config{5}}, "bar"}},
                   "test"};
  const auto data = internal::Visitor::getInfo(test);
  auto info = data.serializeFieldInfos();
  const std::string expected = R"(
type: config
name: MixedConfig
fields:
  - {type: field, name: foo, value: test, default: bar, input_info: {type: string}}
  - type: config
    name: BaseImpl
    field_name: base
    available_types: [BaseImpl]
    fields:
      - {type: field, name: a, value: 1, default: 1, input_info: {type: float32}}
  - type: config
    name: ParentImpl
    field_name: parent
    available_types: [ParentImpl, Uninitialized Virtual Config]
    fields:
      - {type: field, name: other, value: 'bar', default: '', input_info: {type: string}}
      - type: config
        name: BaseImpl
        field_name: base
        available_types: [BaseImpl]
        fields:
          - {type: field, name: a, value: 5, default: 1, input_info: {type: float32}}
)";

  expectEqual(info, YAML::Load(expected), 1e-6);
}

}  // namespace config::test
