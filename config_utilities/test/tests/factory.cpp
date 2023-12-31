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

TEST(Factory, create) {
  std::unique_ptr<Base> base = create<Base>("DerivedA", 1);
  EXPECT_TRUE(base);
  EXPECT_EQ(base->name(), "DerivedA");

  base = create<Base>("DerivedB", 1);
  EXPECT_TRUE(base);
  EXPECT_EQ(base->name(), "DerivedB");

  auto logger = TestLogger::create();
  base = create<Base>("NotRegistered", 1);
  EXPECT_FALSE(base);
  ASSERT_EQ(logger->numMessages(), 1);
  std::string msg = logger->messages().back().second;
  EXPECT_EQ(msg.find("No module of type 'NotRegistered' registered to the factory"), 0);
  EXPECT_NE(msg.find("Registered are: 'DerivedB', 'DerivedA'."), std::string::npos);

  base = create<Base>("DerivedA", 1, 2.f);
  EXPECT_FALSE(base);
  ASSERT_GE(logger->numMessages(), 2);
  msg = logger->messages().at(1).second;
  EXPECT_EQ(msg.find("Cannot create a module of type 'DerivedA': No modules registered to the factory"), 0);
  EXPECT_NE(
      msg.find(
          "Register modules using a static config::Registration<BaseT, DerivedT, ConstructorArguments...> struct."),
      std::string::npos);
}

TEST(Factory, createWithConfig) {
  YAML::Node data;
  data["i"] = 3;
  data["f"] = 3.14f;
  data["type"] = "DerivedC";

  std::unique_ptr<Base> base = createFromYaml<Base>(data, 12);
  EXPECT_TRUE(base);
  EXPECT_EQ(base->name(), "DerivedC");
  EXPECT_EQ(dynamic_cast<DerivedC*>(base.get())->config_.f, 3.14f);
  EXPECT_EQ(base->i_, 12);

  auto logger = TestLogger::create();
  data["type"] = "NotRegistered";
  base = createFromYaml<Base>(data, 12);
  EXPECT_FALSE(base);
  EXPECT_EQ(logger->numMessages(), 1);
  std::string msg = logger->messages().back().second;
  EXPECT_EQ(msg.find("No module of type 'NotRegistered' registered to the factory"), 0);

  Settings().factory_type_param_name = "test_type";
  base = createFromYaml<Base>(data, 12);
  EXPECT_FALSE(base);
  EXPECT_EQ(logger->numMessages(), 2);
  msg = logger->messages().back().second;
  EXPECT_EQ(msg, "Could not read the param 'test_type' to deduce the type of the module to create.");

  data["test_type"] = "DerivedD";
  base = createFromYaml<Base>(data, 12);
  EXPECT_TRUE(base);
  EXPECT_EQ(base->name(), "DerivedD");
  EXPECT_EQ(dynamic_cast<DerivedD*>(base.get())->config_.i, 3);
}

}  // namespace config::test
