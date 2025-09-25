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

#include "config_utilities/virtual_config.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/formatting/asl.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/printing.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/validation.h"

namespace config::test {

struct Base2 {
  virtual std::string name() const = 0;
  virtual ~Base2() = default;
};

struct Derived2 : public Base2 {
  struct Config {
    float f = 0.f;
    std::string s = "test string";
  };
  explicit Derived2(const Config& config) : config_(config) {}
  std::string name() const override { return "Derived2"; }
  const Config config_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base2, Derived2, Derived2::Config>("Derived2");
};

void declare_config(Derived2::Config& config) {
  config::name("Derived2");
  config::field(config.f, "f", "m/s");
  config::field(config.s, "s");
  config::check(config.f, config::CheckMode::GE, 0.f, "f");
}

struct Derived2A : public Base2 {
  struct Config {
    int i = 0;
  };
  explicit Derived2A(const Config& config) : config_(config) {}
  std::string name() const override { return "Derived2A"; }
  const Config config_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base2, Derived2A, Derived2A::Config>("Derived2A");
};

void declare_config(Derived2A::Config& config) {
  config::name("Derived2A");
  config::field(config.i, "i");
}

struct Derived2WithComplexParam : public Base2 {
  struct Config {
    int i = 0;
  };
  explicit Derived2WithComplexParam(const Config& config, std::shared_ptr<int> i) : config_(config), i_(i) {}
  std::string name() const override { return "Derived2WithComplexParam"; }
  const Config config_;
  const std::shared_ptr<int> i_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base2, Derived2WithComplexParam, Config, std::shared_ptr<int>>(
          "Derived2WithComplexParam");
};

void declare_config(Derived2WithComplexParam::Config& config) {
  config::name("Derived2WithComplexParam");
  config::field(config.i, "i");
}

struct Derived2WithMoveOnlyParam : public Base2 {
  struct Config {
    int i = 0;
  };
  explicit Derived2WithMoveOnlyParam(const Config& config, std::unique_ptr<int> i)
      : config_(config), i_(std::move(i)) {}
  std::string name() const override { return "Derived2WithMoveOnlyParam"; }
  const Config config_;
  const std::unique_ptr<int> i_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base2, Derived2WithMoveOnlyParam, Config, std::unique_ptr<int>>(
          "Derived2WithMoveOnlyParam");
};

void declare_config(Derived2WithMoveOnlyParam::Config& config) {
  config::name("Derived2WithMoveOnlyParam");
  config::field(config.i, "i");
}

struct NotDerivedFromBase2 {
  struct Config {
    bool b = false;
    VirtualConfig<Base2> base_config{Derived2::Config()};
  };
};

void declare_config(NotDerivedFromBase2::Config& config) {
  config::name("NotDerivedFromBase2");
  config::field(config.b, "b");
  config::field(config.base_config, "base_config");
}

struct ObjectWithBase {
  struct Config {
    double d = 0.0;
    VirtualConfig<Base2> base_config;
  };
  explicit ObjectWithBase(const Config& config) : config_(config) { base_ = config_.base_config.create(); }
  const Config config_;
  std::unique_ptr<Base2> base_;
};

void declare_config(ObjectWithBase::Config& config) {
  // Declare the config using the config utilities.
  config::name("ObjectWithBase");
  config::field(config.d, "d", "kg/m^3");
  config::check(config.d, config::CheckMode::GE, 0.0, "d");
  config::field(config.base_config, "base_config", false);
}

struct ObjectWithOptionalConfigs {
  struct Config {
    std::vector<VirtualConfig<Base2, true>> modules;
  } const config;

  explicit ObjectWithOptionalConfigs(const Config& config) : config(config::checkValid(config)) {
    for (const auto& base_config : config.modules) {
      if (base_config) {
        valid.emplace_back(base_config.create());
      }
    }
  }

  std::vector<std::unique_ptr<Base2>> valid;
};

void declare_config(ObjectWithOptionalConfigs::Config& config) {
  config::name("ObjectWithOptionalConfigs");
  config::field(config.modules, "modules");
}

struct DefaultedOptional {
  struct Config {
    int foo = 3;
  } const config;
  explicit DefaultedOptional(const Config& config) : config(config) {}
};

void declare_config(DefaultedOptional::Config& config) {
  name<DefaultedOptional::Config>();
  field(config.foo, "foo");
}

struct ParentOfDefaultedOptional {
  struct Config {
    VirtualConfig<DefaultedOptional> child{DefaultedOptional::Config()};
  } const config;

  explicit ParentOfDefaultedOptional(const Config& config) : config(config), child(config.child.create()) {}
  std::unique_ptr<DefaultedOptional> child;
};

void declare_config(ParentOfDefaultedOptional::Config& config) {
  name<ParentOfDefaultedOptional::Config>();
  field(config.child, "child");
  config.child.setOptional();
}

struct GrandparentOfDefaultedOptional {
  struct Config {
    VirtualConfig<ParentOfDefaultedOptional> child;
  } const config;

  explicit GrandparentOfDefaultedOptional(const Config& config) : config(config), child(config.child.create()) {}
  std::unique_ptr<ParentOfDefaultedOptional> child;
};

void declare_config(GrandparentOfDefaultedOptional::Config& config) {
  name<ParentOfDefaultedOptional::Config>();
  field(config.child, "child");
  config.child.setOptional();
}

TEST(VirtualConfig, isSet) {
  Settings().restoreDefaults();

  VirtualConfig<Base2> config;
  EXPECT_FALSE(config.isSet());
  EXPECT_FALSE(isValid(config));

  YAML::Node data;
  config = fromYaml<VirtualConfig<Base2>>(data);
  EXPECT_FALSE(config.isSet());
  EXPECT_FALSE(isValid(config));

  data["type"] = "Derived2";
  config = fromYaml<VirtualConfig<Base2>>(data);
  EXPECT_TRUE(config.isSet());
  EXPECT_TRUE(isValid(config));
  EXPECT_EQ(config.getType(), "Derived2");
}

TEST(VirtualConfig, copyMove) {
  VirtualConfig<Base2> config;
  VirtualConfig<Base2> other;

  config = other;
  EXPECT_FALSE(config.isSet());

  config = std::move(other);
  EXPECT_FALSE(config.isSet());

  VirtualConfig<Base2> config2(config);
  EXPECT_FALSE(config2.isSet());

  VirtualConfig<Base2> config3(std::move(config));
  EXPECT_FALSE(config3.isSet());

  YAML::Node data;
  data["type"] = "Derived2";
  other = fromYaml<VirtualConfig<Base2>>(data);

  config = other;
  EXPECT_TRUE(config.isSet());
  EXPECT_EQ(config.getType(), "Derived2");

  config = std::move(other);
  EXPECT_TRUE(config.isSet());
  EXPECT_EQ(config.getType(), "Derived2");

  VirtualConfig<Base2> config4(config);
  EXPECT_TRUE(config4.isSet());
  EXPECT_EQ(config4.getType(), "Derived2");

  VirtualConfig<Base2> config5(std::move(config));
  EXPECT_TRUE(config5.isSet());
  EXPECT_EQ(config5.getType(), "Derived2");
}

TEST(VirtualConfig, assignConfig) {
  Derived2::Config derived_config;
  derived_config.f = -1.f;
  derived_config.s = "abcdef";

  // Set a virtual config to a specified config.
  VirtualConfig<Base2> config;
  EXPECT_FALSE(config.isSet());
  config.set(derived_config);
  EXPECT_TRUE(config.isSet());
  EXPECT_EQ(config.getType(), "Derived2");
  auto derived = config.create();
  EXPECT_TRUE(derived);
  EXPECT_EQ(derived->name(), "Derived2");
  const auto& realized_config = dynamic_cast<Derived2*>(derived.get())->config_;
  EXPECT_EQ(realized_config.f, -1.f);
  EXPECT_EQ(realized_config.s, "abcdef");

  // Operator= should overwrite the config.
  Derived2A::Config derived_config_a;
  derived_config_a.i = 1;
  config = derived_config_a;
  EXPECT_TRUE(config.isSet());
  EXPECT_EQ(config.getType(), "Derived2A");
  derived = config.create();
  EXPECT_TRUE(derived);
  EXPECT_EQ(derived->name(), "Derived2A");
  const auto& realized_config_a = dynamic_cast<Derived2A*>(derived.get())->config_;
  EXPECT_EQ(realized_config_a.i, 1);

  // Constructor and list initialization.
  VirtualConfig<Base2> config2(derived_config);
  EXPECT_TRUE(config2.isSet());
  EXPECT_EQ(config2.getType(), "Derived2");

  NotDerivedFromBase2::Config not_derived_config;
  EXPECT_TRUE(not_derived_config.base_config.isSet());
  EXPECT_EQ(not_derived_config.base_config.getType(), "Derived2");

  // Fail to set a config that is not registered.
  auto logger = TestLogger::create();
  VirtualConfig<Base2> config3;
  const bool success = config3.set(not_derived_config);
  EXPECT_FALSE(success);
  EXPECT_FALSE(config3.isSet());
  EXPECT_EQ(logger->numMessages(), 1);
  EXPECT_EQ(logger->messages()[0].second,
            "No module for config 'config::test::NotDerivedFromBase2::Config' is registered to the factory for "
            "'config::test::Base2' to set virtual config.");
}

TEST(VirtualConfig, create) {
  {
    VirtualConfig<Base2> config;
    std::unique_ptr<Base2> object = config.create();
    EXPECT_FALSE(object);
  }
  {
    YAML::Node data;
    data["type"] = "Derived2";
    data["f"] = 1.f;
    auto config = fromYaml<VirtualConfig<Base2>>(data);
    auto object = config.create();
    EXPECT_TRUE(object);
    EXPECT_EQ(object->name(), "Derived2");
    EXPECT_EQ(dynamic_cast<Derived2*>(object.get())->config_.f, 1.f);
  }
  {
    // Create an object with a parameter
    YAML::Node data;
    data["type"] = "Derived2WithComplexParam";
    data["i"] = 1234;
    auto config = fromYaml<VirtualConfig<Base2>>(data);
    {
      // Create by l-value
      auto i = std::make_shared<int>(5678);
      auto object = config.create(i);
      EXPECT_TRUE(object);
      EXPECT_EQ(object->name(), "Derived2WithComplexParam");
      auto ptr = dynamic_cast<Derived2WithComplexParam*>(object.get());
      ASSERT_NE(ptr, nullptr);
      EXPECT_EQ(ptr->config_.i, 1234);
      EXPECT_EQ(*ptr->i_, 5678);
    }
    {
      // Create by const l-value
      const auto i = std::make_shared<int>(5678);
      auto object = config.create(i);
      EXPECT_TRUE(object);
      EXPECT_EQ(object->name(), "Derived2WithComplexParam");
      auto ptr = dynamic_cast<Derived2WithComplexParam*>(object.get());
      ASSERT_NE(ptr, nullptr);
      EXPECT_EQ(ptr->config_.i, 1234);
      EXPECT_EQ(*ptr->i_, 5678);
    }
    {
      // Create by r-value
      auto object = config.create(std::make_shared<int>(5678));
      EXPECT_TRUE(object);
      EXPECT_EQ(object->name(), "Derived2WithComplexParam");
      auto ptr = dynamic_cast<Derived2WithComplexParam*>(object.get());
      ASSERT_NE(ptr, nullptr);
      EXPECT_EQ(ptr->config_.i, 1234);
      EXPECT_EQ(*ptr->i_, 5678);
    }
  }
  {
    // Create an object with a move-only parameter
    YAML::Node data;
    data["type"] = "Derived2WithMoveOnlyParam";
    data["i"] = 4321;
    auto config = fromYaml<VirtualConfig<Base2>>(data);
    {
      // Create by l-value
      auto i = std::make_unique<int>(8765);
      auto object = config.create(std::move(i));
      ASSERT_TRUE(object);
      EXPECT_EQ(object->name(), "Derived2WithMoveOnlyParam");
      auto ptr = dynamic_cast<Derived2WithMoveOnlyParam*>(object.get());
      ASSERT_NE(ptr, nullptr);
      EXPECT_EQ(ptr->config_.i, 4321);
      EXPECT_EQ(*ptr->i_, 8765);
    }
    {
      // Create by r-value
      auto object = config.create(std::make_unique<int>(8765));
      ASSERT_TRUE(object);
      EXPECT_EQ(object->name(), "Derived2WithMoveOnlyParam");
      auto ptr = dynamic_cast<Derived2WithMoveOnlyParam*>(object.get());
      ASSERT_NE(ptr, nullptr);
      EXPECT_EQ(ptr->config_.i, 4321);
      EXPECT_EQ(*ptr->i_, 8765);
    }
    {
      // Create incorrectly
      auto i = std::make_unique<const int>(8765);  // The const should create a type mismatch
      auto object = config.create(std::move(i));
      EXPECT_FALSE(object);
    }
  }
}

TEST(VirtualConfig, isOptional) {
  VirtualConfig<Base2> config;
  auto logger = TestLogger::create();
  EXPECT_FALSE(isValid(config, true));
  EXPECT_EQ(logger->numMessages(), 1);
  std::string msg = logger->messages()[0].second;
  std::string expected = R"""(Invalid config 'Uninitialized Virtual Config':
========================= Uninitialized Virtual Config =========================
Warning: Check [1/1] failed: Virtual config is not set and not marked optional.
================================================================================)""";
  EXPECT_EQ(msg, expected);

  config.setOptional();
  EXPECT_TRUE(isValid(config));
}

TEST(VirtualConfig, defaultOptional) {
  {
    constexpr bool kDefaultOptional = true;
    VirtualConfig<Base2, kDefaultOptional> config;
    EXPECT_TRUE(isValid(config));
  }
  {
    constexpr bool kDefaultOptional = false;
    VirtualConfig<Base2, kDefaultOptional> config;
    EXPECT_FALSE(isValid(config));
  }
}

TEST(VirtualConfig, printing) {
  VirtualConfig<Base2> config;
  std::string msg = toString(config);
  std::string expected = R"""(========================= Uninitialized Virtual Config =========================
================================================================================)""";
  EXPECT_EQ(msg, expected);

  YAML::Node data;
  data["type"] = "Derived2";
  data["f"] = 1.f;
  data["s"] = "test string";
  config = fromYaml<VirtualConfig<Base2>>(data);

  EXPECT_TRUE(isValid(config));
  msg = toString(config);
  expected = R"""(=========================== Virtual Config: Derived2 ===========================
f [m/s]:                      1
s:                            test string (default)
================================================================================)""";
  EXPECT_EQ(msg, expected);

  auto logger = TestLogger::create();
  data["f"] = -1.f;
  config = fromYaml<VirtualConfig<Base2>>(data);
  EXPECT_FALSE(isValid(config, true));
  EXPECT_EQ(logger->numMessages(), 1);
  msg = logger->messages()[0].second;
  expected = R"""(Invalid config 'Virtual Config: Derived2':
=========================== Virtual Config: Derived2 ===========================
Warning: Check [2/2] failed for 'f': param >= 0 (is: '-1').
================================================================================)""";
  EXPECT_EQ(msg, expected);
}

TEST(VirtualConfig, subconfig) {
  ObjectWithBase::Config config;
  config.d = 1.0;
  std::string msg = toString(config);
  std::string expected = R"""(================================ ObjectWithBase ================================
d [kg/m^3]:                   1
base_config [Uninitialized Virtual Config]
================================================================================)""";
  EXPECT_EQ(msg, expected);
  EXPECT_FALSE(isValid(config));

  YAML::Node data;
  data["d"] = 1.0;
  data["type"] = "Derived2";
  data["f"] = 1.f;
  config = fromYaml<ObjectWithBase::Config>(data);
  EXPECT_TRUE(isValid(config));
  EXPECT_EQ(config.base_config.getType(), "Derived2");

  msg = toString(config);
  expected = R"""(================================ ObjectWithBase ================================
d [kg/m^3]:                   1
base_config [Virtual Config: Derived2]:
   f [m/s]:                   1
   s:                         test string (default)
================================================================================)""";
  EXPECT_EQ(msg, expected);

  data["f"] = -1.f;
  config = fromYaml<ObjectWithBase::Config>(data);
  auto logger = TestLogger::create();
  EXPECT_FALSE(isValid(config, true));
  EXPECT_EQ(logger->numMessages(), 1);
  msg = logger->messages()[0].second;
  expected = R"""(Invalid config 'ObjectWithBase':
================================ ObjectWithBase ================================
Warning: Check [3/3] failed for 'base_config.f': param >= 0 (is: '-1').
================================================================================)""";
  EXPECT_EQ(msg, expected);
}

TEST(VirtualConfig, getUnderlying) {
  Settings().restoreDefaults();

  VirtualConfig<Base2> config;
  EXPECT_FALSE(config.isSet());
  EXPECT_FALSE(config.getUnderlying<Derived2>());

  config = Derived2::Config();
  EXPECT_TRUE(config.isSet());
  EXPECT_FALSE(config.getUnderlying<Derived2A::Config>());
  EXPECT_TRUE(config.getUnderlying<Derived2::Config>());

  config = Derived2A::Config();
  EXPECT_TRUE(config.isSet());
  EXPECT_FALSE(config.getUnderlying<Derived2::Config>());
  EXPECT_TRUE(config.getUnderlying<Derived2A::Config>());
}

TEST(VirtualConfig, optionalByDefault) {
  Settings().restoreDefaults();

  const std::string yaml_str = R"""(modules:
  - {type: testing}
  - {a: 5, b: 6}
  - {type: Derived2}
)""";

  const auto node = YAML::Load(yaml_str);
  const auto config = config::fromYaml<ObjectWithOptionalConfigs::Config>(node);
  const ObjectWithOptionalConfigs object(config);
  EXPECT_EQ(object.valid.size(), 1u);
}

TEST(VirtualConfig, parseOptionalMissingField) {
  {
    constexpr bool kDefaultOptional = true;
    YAML::Node data;
    auto config = fromYaml<VirtualConfig<Base2, kDefaultOptional>>(data);
    EXPECT_TRUE(isValid(config));
    EXPECT_TRUE(config.optional());
  }
  {
    constexpr bool kDefaultOptional = false;
    YAML::Node data;
    auto config = fromYaml<VirtualConfig<Base2, kDefaultOptional>>(data);
    EXPECT_FALSE(isValid(config));
    EXPECT_FALSE(config.optional());
  }
}

TEST(VirtualConfig, parseOptionalFieldPresent) {
  YAML::Node data;
  data["type"] = "Derived2";
  data["f"] = 1.f;
  data["s"] = "blahblah";

  {
    constexpr bool kDefaultOptional = true;
    auto config = fromYaml<VirtualConfig<Base2, kDefaultOptional>>(data);
    EXPECT_TRUE(isValid(config));
    EXPECT_TRUE(config.isSet());
    EXPECT_EQ(config.getType(), "Derived2");
    EXPECT_TRUE(config.optional());

    auto actual_config = config.getUnderlying<Derived2::Config>();
    ASSERT_TRUE(actual_config);
    EXPECT_FLOAT_EQ(actual_config->f, 1.f);
    EXPECT_EQ(actual_config->s, "blahblah");
  }
  {
    constexpr bool kDefaultOptional = false;
    auto config = fromYaml<VirtualConfig<Base2, kDefaultOptional>>(data);
    EXPECT_TRUE(isValid(config));
    EXPECT_TRUE(config.isSet());
    EXPECT_EQ(config.getType(), "Derived2");
    EXPECT_FALSE(config.optional());

    auto actual_config = config.getUnderlying<Derived2::Config>();
    ASSERT_TRUE(actual_config);
    EXPECT_FLOAT_EQ(actual_config->f, 1.f);
    EXPECT_EQ(actual_config->s, "blahblah");
  }
}

TEST(VirtualConfig, optionalNullCreation) {
  YAML::Node data;
  constexpr bool kDefaultOptional = true;
  auto config = fromYaml<VirtualConfig<Base2, kDefaultOptional>>(data);
  ASSERT_TRUE(isValid(config));

  auto object = config.create();
  ASSERT_EQ(object, nullptr);
}

TEST(VirtualConfig, defaultedConfigCorrect) {
  RegistrationGuard<DefaultedOptional, DefaultedOptional, DefaultedOptional::Config> guard("DefaultedOptional");
  RegistrationGuard<ParentOfDefaultedOptional, ParentOfDefaultedOptional, ParentOfDefaultedOptional::Config>
      parent_guard("ParentOfDefaultedOptional");
  RegistrationGuard<GrandparentOfDefaultedOptional,
                    GrandparentOfDefaultedOptional,
                    GrandparentOfDefaultedOptional::Config>
      grandparent_guard("GrandparentOfDefaultedOptional");

  {  // default config does the right thing from YAML
    const auto node = YAML::Load(R"""(
type: GrandparentOfDefaultedOptional
child:
  type: ParentOfDefaultedOptional
)""");
    auto root = config::createFromYaml<GrandparentOfDefaultedOptional>(node);
    ASSERT_TRUE(root);
    ASSERT_TRUE(root->child);
    EXPECT_TRUE(root->child->child);
  }

  {  // manually specifying the type does the right thing
    const auto node = YAML::Load(R"""(
type: GrandparentOfDefaultedOptional
child:
  type: ParentOfDefaultedOptional
  child:
    type: DefaultedOptional
    foo: 5
)""");
    auto root = config::createFromYaml<GrandparentOfDefaultedOptional>(node);
    ASSERT_TRUE(root);
    ASSERT_TRUE(root->child);
    ASSERT_TRUE(root->child->child);
    EXPECT_EQ(root->child->child->config.foo, 5);
  }

  {  // overriding default does the right thing
    const auto node = YAML::Load(R"""(
type: GrandparentOfDefaultedOptional
child:
  type: ParentOfDefaultedOptional
  child:
    type: ''
)""");
    auto root_config = config::fromYaml<GrandparentOfDefaultedOptional::Config>(node);
    auto root = config::createFromYaml<GrandparentOfDefaultedOptional>(node);
    ASSERT_TRUE(root);
    ASSERT_TRUE(root->child);
    EXPECT_FALSE(root->child->child);
  }
}

TEST(VirtualConfig, equality) {
  VirtualConfig<Base2> config1;
  VirtualConfig<Base2> config2;
  EXPECT_TRUE(config1 == config2);

  config1 = Derived2::Config();
  EXPECT_FALSE(config1 == config2);

  config2 = Derived2::Config();
  EXPECT_TRUE(config1 == config2);

  config1 = Derived2A::Config();
  EXPECT_FALSE(config1 == config2);

  auto derived_cfg = Derived2A::Config();
  derived_cfg.i = 5;
  config2 = derived_cfg;
  EXPECT_FALSE(config1 == config2);
}

TEST(VirtualConfig, inequality) {
  VirtualConfig<Base2> config1;
  VirtualConfig<Base2> config2;
  EXPECT_FALSE(config1 != config2);

  config1 = Derived2::Config();
  EXPECT_TRUE(config1 != config2);

  config2 = Derived2::Config();
  EXPECT_FALSE(config1 != config2);

  config1 = Derived2A::Config();
  EXPECT_TRUE(config1 != config2);

  auto derived_cfg = Derived2A::Config();
  derived_cfg.i = 5;
  config2 = derived_cfg;
  EXPECT_TRUE(config1 != config2);
}

}  // namespace config::test
