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

#include "config_utilities/factory.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/logging/log_to_stdout.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/traits.h"

namespace config::test {

class Base {
 public:
  explicit Base(int i) : i_(i) {}
  virtual ~Base() = default;
  const int i_;
  virtual std::string name() const = 0;
};

class DerivedA : public Base {
 public:
  explicit DerivedA(int i) : Base(i) {}
  std::string name() const override { return "DerivedA"; }
  inline static const auto registration_ = config::Registration<Base, DerivedA, int>("DerivedA");
};

class DerivedB : public Base {
 public:
  explicit DerivedB(int i) : Base(i) {}
  std::string name() const override { return "DerivedB"; }
  inline static const auto registration_ = config::Registration<Base, DerivedB, int>("DerivedB");
};

class DerivedC : public Base {
 public:
  struct Config {
    float f = 0.f;
  };
  DerivedC(const Config& config, const int& i) : Base(i), config_(config) {}
  std::string name() const override { return "DerivedC"; }
  const Config config_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base, DerivedC, DerivedC::Config, int>("DerivedC");
};

void declare_config(DerivedC::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedC");
  config::field(config.f, "f");
}

class DerivedD : public Base {
 public:
  struct Config {
    int i = 0;
  };
  DerivedD(const Config& config, const int& i) : Base(i), config_(config) {}
  std::string name() const override { return "DerivedD"; }
  const Config config_;
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base, DerivedD, DerivedD::Config, int>("DerivedD");
};

void declare_config(DerivedD::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedD");
  config::field(config.i, "i");
}

class DerivedWithMoveOnlyParameter : public Base {
 public:
  explicit DerivedWithMoveOnlyParameter(std::unique_ptr<int> i) : Base(*i), i_(std::move(i)) {}
  std::string name() const override { return "DerivedWithMoveOnlyParameter"; }

  const std::unique_ptr<int> i_;

 private:
  inline static const auto registration_ =
      config::Registration<Base, DerivedWithMoveOnlyParameter, std::unique_ptr<int>>("DerivedWithMoveOnlyParameter");
};

class DerivedWithComplexParameter : public Base {
 public:
  explicit DerivedWithComplexParameter(std::shared_ptr<int> i) : Base(*i), i_(std::move(i)) {}
  std::string name() const override { return "DerivedWithComplexParameter"; }

  std::shared_ptr<int> i_;

 private:
  inline static const auto registration_ =
      config::Registration<Base, DerivedWithComplexParameter, std::shared_ptr<int>>("DerivedWithComplexParameter");
};

class DerivedWithMoveOnlyParameterAndConfig : public Base {
 public:
  struct Config {
    float f = 123.f;
  };

  DerivedWithMoveOnlyParameterAndConfig(const Config& config, std::unique_ptr<int> i)
      : Base(*i), i_(std::move(i)), config_(config) {}
  std::string name() const override { return "DerivedWithMoveOnlyParameterAndConfig"; }

  const std::unique_ptr<int> i_;
  Config config_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base, DerivedWithMoveOnlyParameterAndConfig, Config, std::unique_ptr<int>>(
          "DerivedWithMoveOnlyParameterAndConfig");
};

void declare_config(DerivedWithMoveOnlyParameterAndConfig::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedWithMoveOnlyParameterAndConfig");
  config::field(config.f, "f");
}

template <typename T>
struct TemplatedBase {
  virtual ~TemplatedBase() = default;
};

template <typename DerivedT, typename BaseT>
struct TemplatedDerived : public TemplatedBase<BaseT> {
  explicit TemplatedDerived(bool b = true) : b_(b) {}
  const bool b_;
};

TEST(Factory, moduleInfo) {
  {  // first arg is valid
    const auto info = internal::ModuleInfo::fromTypes<void, int, float, double, char>();
    EXPECT_EQ(info.argumentString(), "int, float, double, char");
    EXPECT_EQ(info.argumentString(" | ", "'"), "'int' | 'float' | 'double' | 'char'");
    EXPECT_EQ(info.argumentString(", ", "", "_"), "int, float, double, char");
    EXPECT_EQ(info.argumentString(", ", "'", "*"), "'int', 'float', 'double', 'char'");
  }

  {  // first arg is skipped
    const auto info = internal::ModuleInfo::fromTypes<void, int, float, double, char>(true);
    EXPECT_EQ(info.argumentString(), "float, double, char");
    EXPECT_EQ(info.argumentString(" | ", "'"), "'float' | 'double' | 'char'");
    EXPECT_EQ(info.argumentString(", ", "", "_"), "_, float, double, char");
    EXPECT_EQ(info.argumentString(", ", "'", "*"), "*, 'float', 'double', 'char'");
  }
}

TEST(Factory, create) {
  {
    auto base = create<Base>("DerivedA", 1);
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedA");
  }
  {
    // Create b with an r-value
    auto base = create<Base>("DerivedB", 1);
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedB");
  }
  {
    // Create b with an l-value
    int i = 1;
    auto base = create<Base>("DerivedB", i);
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedB");
  }
  {
    // Create b with an r-value reference
    int i = 1;
    auto base = create<Base>("DerivedB", std::move(i));
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedB");
  }
  {
    // Try to create an object that is not registered
    auto logger = TestLogger::create();
    auto base = create<Base>("NotRegistered", 1);
    EXPECT_FALSE(base);
    ASSERT_EQ(logger->numMessages(), 1);
    std::string msg = logger->messages().back().second;
    EXPECT_EQ(msg.find("No module of type 'NotRegistered' registered to the factory"), 0);
    EXPECT_NE(msg.find("Registered are: 'DerivedA', 'DerivedB'."), std::string::npos) << msg;
  }
  {
    // Try to create an object that is registered but with the wrong arguments
    auto logger = TestLogger::create();
    auto base = create<Base>("DerivedA", 1, 2.f);
    EXPECT_FALSE(base);
    ASSERT_EQ(logger->numMessages(), 1);
    auto msg = logger->messages().back().second;
    EXPECT_EQ(msg.find("Cannot create a module of type 'DerivedA': No modules registered to the factory"), 0);
    EXPECT_NE(
        msg.find(
            "Register modules using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct."),
        std::string::npos);
  }
  {
    // Try to create an object that takes a move-only parameter r-value
    auto base = create<Base>("DerivedWithMoveOnlyParameter", std::make_unique<int>(1));
    EXPECT_TRUE(base);
    EXPECT_EQ(base->i_, 1);
    EXPECT_EQ(base->name(), "DerivedWithMoveOnlyParameter");
    auto ptr = dynamic_cast<DerivedWithMoveOnlyParameter*>(base.get());
    ASSERT_NE(ptr, nullptr);
    EXPECT_EQ(*ptr->i_, 1);
  }
  {
    // Try to create an object that takes a move-only parameter l-value
    auto i = std::make_unique<int>(1);
    auto base = create<Base>("DerivedWithMoveOnlyParameter", std::move(i));
    EXPECT_TRUE(base);
    EXPECT_EQ(base->i_, 1);
    EXPECT_EQ(base->name(), "DerivedWithMoveOnlyParameter");
    auto ptr = dynamic_cast<DerivedWithMoveOnlyParameter*>(base.get());
    ASSERT_NE(ptr, nullptr);
    EXPECT_EQ(*ptr->i_, 1);
  }
  {
    // Try to create an object that takes a complex parameter by copy from an l-value
    auto i = std::make_shared<int>(1);
    auto base = create<Base>("DerivedWithComplexParameter", i);
    EXPECT_TRUE(base);
    EXPECT_EQ(base->i_, 1);
    EXPECT_EQ(base->name(), "DerivedWithComplexParameter");
    auto ptr = dynamic_cast<DerivedWithComplexParameter*>(base.get());
    ASSERT_NE(ptr, nullptr);
  }
  {
    // Try to create an object that takes a complex parameter by copy from an r-value
    auto base = create<Base>("DerivedWithComplexParameter", std::make_shared<int>(1));
    EXPECT_TRUE(base);
    EXPECT_EQ(base->i_, 1);
    EXPECT_EQ(base->name(), "DerivedWithComplexParameter");
    auto ptr = dynamic_cast<DerivedWithComplexParameter*>(base.get());
    ASSERT_NE(ptr, nullptr);
  }
  {
    // Try to create an object that takes a complex parameter using an r-value reference
    auto i = std::make_shared<int>(1);
    auto base = create<Base>("DerivedWithComplexParameter", std::move(i));
    EXPECT_TRUE(base);
    EXPECT_EQ(base->i_, 1);
    EXPECT_EQ(base->name(), "DerivedWithComplexParameter");
    auto ptr = dynamic_cast<DerivedWithComplexParameter*>(base.get());
    ASSERT_NE(ptr, nullptr);
  }
}

TEST(Factory, createWithConfig) {
  YAML::Node data;
  data["i"] = 3;
  data["f"] = 3.14f;
  data["type"] = "DerivedC";
  {
    // Create DerivedC with config and r-value parameter
    auto base = createFromYaml<Base>(data, 12);
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedC");
    EXPECT_EQ(dynamic_cast<DerivedC*>(base.get())->config_.f, 3.14f);
    EXPECT_EQ(base->i_, 12);
  }
  {
    // Create DerivedC with config and l-value parameter
    int i = 12;
    auto base = createFromYaml<Base>(data, i);
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedC");
    EXPECT_EQ(dynamic_cast<DerivedC*>(base.get())->config_.f, 3.14f);
    EXPECT_EQ(base->i_, 12);
  }
  {
    // Create DerivedC with config and const l-value parameter
    const int i = 12;
    auto base = createFromYaml<Base>(data, i);
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedC");
    EXPECT_EQ(dynamic_cast<DerivedC*>(base.get())->config_.f, 3.14f);
    EXPECT_EQ(base->i_, 12);
  }
  {
    // Create DerivedC with config and an r-value reference
    int i = 12;
    auto base = createFromYaml<Base>(data, std::move(i));
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedC");
    EXPECT_EQ(dynamic_cast<DerivedC*>(base.get())->config_.f, 3.14f);
    EXPECT_EQ(base->i_, 12);
  }
  {
    auto logger = TestLogger::create();
    data["type"] = "NotRegistered";
    auto base = createFromYaml<Base>(data, 12);
    EXPECT_FALSE(base);
    EXPECT_EQ(logger->numMessages(), 1);
    std::string msg = logger->messages().back().second;
    EXPECT_EQ(msg.find("No module of type 'NotRegistered' registered to the factory"), 0);
  }
  {
    auto logger = TestLogger::create();
    Settings().factory.type_param_name = "test_type";
    auto base = createFromYaml<Base>(data, 12);
    EXPECT_FALSE(base);
    EXPECT_EQ(logger->numMessages(), 1);
    auto msg = logger->messages().back().second;
    EXPECT_EQ(msg, "Could not read the param 'test_type' to deduce the type of the module to create.");
  }
  {
    data["test_type"] = "DerivedD";
    auto base = createFromYaml<Base>(data, 12);
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedD");
    EXPECT_EQ(dynamic_cast<DerivedD*>(base.get())->config_.i, 3);
  }
  {
    // Build with r-value move-only parameter
    Settings().restoreDefaults();
    data["type"] = "DerivedWithMoveOnlyParameterAndConfig";
    auto base = createFromYaml<Base>(data, std::make_unique<int>(1));
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedWithMoveOnlyParameterAndConfig");
    auto ptr = dynamic_cast<DerivedWithMoveOnlyParameterAndConfig*>(base.get());
    ASSERT_NE(ptr, nullptr);
    EXPECT_FLOAT_EQ(ptr->config_.f, 3.14f);
    EXPECT_EQ(*ptr->i_, 1);
  }
  {
    // Build with l-value move-only parameter
    data["type"] = "DerivedWithMoveOnlyParameterAndConfig";
    auto i = std::make_unique<int>(1);
    auto base = createFromYaml<Base>(data, std::move(i));
    EXPECT_TRUE(base);
    EXPECT_EQ(base->name(), "DerivedWithMoveOnlyParameterAndConfig");
    auto ptr = dynamic_cast<DerivedWithMoveOnlyParameterAndConfig*>(base.get());
    ASSERT_NE(ptr, nullptr);
    EXPECT_FLOAT_EQ(ptr->config_.f, 3.14f);
    EXPECT_EQ(*ptr->i_, 1);
  }
}

TEST(Factory, moduleNameConflicts) {
  auto logger = TestLogger::create();

  // Allow shadowing of same name for different module types.
  const auto registration1 = config::Registration<TemplatedBase<int>, TemplatedDerived<int, int>>("name");
  const auto registration2 = config::Registration<TemplatedBase<float>, TemplatedDerived<float, float>>("name");
  EXPECT_EQ(logger->numMessages(), 0);

  // Same derived different name. NOTE(lschmid): This is allowed, not sure if we would want to warn users though.
  const auto registration3 = config::Registration<TemplatedBase<int>, TemplatedDerived<int, int>>("other_name");
  EXPECT_FALSE(logger->hasMessages());

  // Same derived same name. Not allowed. NOTE(lschmid): Could also be an option to make this allowed (skip silently).
  const auto registration4 = config::Registration<TemplatedBase<int>, TemplatedDerived<int, int>>("name");
  ASSERT_EQ(logger->numMessages(), 1);
  EXPECT_EQ(logger->lastMessage(),
            "Cannot register already existent type 'name' for BaseT='config::test::TemplatedBase<int>' and "
            "ConstructorArguments={}.");

  // Different derived same base and same name. Not allowed.
  const auto registration5 = config::Registration<TemplatedBase<int>, TemplatedDerived<float, int>>("name");
  EXPECT_EQ(logger->numMessages(), 2);
  EXPECT_EQ(logger->lastMessage(),
            "Cannot register already existent type 'name' for BaseT='config::test::TemplatedBase<int>' and "
            "ConstructorArguments={}.");

  // Same name, same base but different constructor arguments. Allowed.
  const auto registration6 = config::Registration<TemplatedBase<int>, TemplatedDerived<int, int>, bool>("name");
  EXPECT_EQ(logger->numMessages(), 2);

  // Different constructor args with different name. Allowed but not encouraged.
  const auto registration7 =
      config::Registration<TemplatedBase<float>, TemplatedDerived<float, float>, bool>("different_name");
  EXPECT_EQ(logger->numMessages(), 2);

  // NOTE(nathan): combining printing and name conflicts tests to avoid different behaviors between clang and gcc (and
  // local usage vs ctest). Otherwise sometimes the modules from moduleNameConflicts would get compiled into the
  // registry and sometimes they wouldn't which would lead to inconsistent test behavior...
  const auto registration8 = config::Registration<TemplatedBase<int>, TemplatedDerived<int, int>>("int_derived");
  const auto registration9 =
      config::Registration<TemplatedBase<float>, TemplatedDerived<float, float>>("float_derived");
  const std::string expected = R"""(########################################
#          Registered Objects          #
########################################

config::internal::Formatter():
  'asl' (config::internal::AslFormatter)

config::internal::Logger():
  'stdout' (config::internal::StdoutLogger)
  'test_logger' (config::test::TestLogger)

config::test::Base(int):
  'DerivedA' (config::test::DerivedA)
  'DerivedB' (config::test::DerivedB)

config::test::Base(std::shared_ptr<int>):
  'DerivedWithComplexParameter' (config::test::DerivedWithComplexParameter)

config::test::Base(std::unique_ptr<int, std::default_delete<int> >):
  'DerivedWithMoveOnlyParameter' (config::test::DerivedWithMoveOnlyParameter)

config::test::Talker():
  'internal' (config::test::InternalTalker)

config::test::TemplatedBase<float>():
  'float_derived' (config::test::TemplatedDerived<float, float>)
  'name' (config::test::TemplatedDerived<float, float>)

config::test::TemplatedBase<float>(bool):
  'different_name' (config::test::TemplatedDerived<float, float>)

config::test::TemplatedBase<int>():
  'int_derived' (config::test::TemplatedDerived<int, int>)
  'name' (config::test::TemplatedDerived<int, int>)
  'other_name' (config::test::TemplatedDerived<int, int>)

config::test::TemplatedBase<int>(bool):
  'name' (config::test::TemplatedDerived<int, int>)

########################################
#   Registered Objects with Configs    #
########################################

config::test::Base(int):
  'DerivedC' (config::test::DerivedC)
  'DerivedD' (config::test::DerivedD)

config::test::Base(std::unique_ptr<int, std::default_delete<int> >):
  'DerivedWithMoveOnlyParameterAndConfig' (config::test::DerivedWithMoveOnlyParameterAndConfig)

config::test::Base2():
  'Derived2' (config::test::Derived2)
  'Derived2A' (config::test::Derived2A)

config::test::Base2(std::shared_ptr<int>):
  'Derived2WithComplexParam' (config::test::Derived2WithComplexParam)

config::test::Base2(std::unique_ptr<int, std::default_delete<int> >):
  'Derived2WithMoveOnlyParam' (config::test::Derived2WithMoveOnlyParam)

config::test::ProcessorBase():
  'AddString' (config::test::AddString)

config::test::Talker():
  'repeating' (config::test::RepeatingTalker)

########################################
#          Registered Configs          #
########################################

Config[config::test::Base]():
  'DerivedC' (config::test::DerivedC::Config)
  'DerivedD' (config::test::DerivedD::Config)
  'DerivedWithMoveOnlyParameterAndConfig' (config::test::DerivedWithMoveOnlyParameterAndConfig::Config)

Config[config::test::Base2]():
  'Derived2' (config::test::Derived2::Config)
  'Derived2A' (config::test::Derived2A::Config)
  'Derived2WithComplexParam' (config::test::Derived2WithComplexParam::Config)
  'Derived2WithMoveOnlyParam' (config::test::Derived2WithMoveOnlyParam::Config)

Config[config::test::ProcessorBase]():
  'AddString' (config::test::AddString::Config)

Config[config::test::Talker]():
  'repeating' (config::test::RepeatingTalker::Config)

)""";

  Settings().printing.width = 40;

  // drop glog if it exists to avoid conditional registration problems
  internal::ObjectFactory<internal::Logger>::removeEntry("glog");
  const std::string modules = internal::ModuleRegistry::getAllRegistered();
  EXPECT_EQ(modules, expected);
  Settings().restoreDefaults();
}

}  // namespace config::test
