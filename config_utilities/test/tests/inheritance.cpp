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

#include <map>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/formatting/asl.h"
#include "config_utilities/printing.h"
#include "config_utilities/test/utils.h"

namespace config::test {

struct Base {
  int i = 1;
};

void declare_config(Base& config) {
  using namespace config;
  name("Base");
  field(config.i, "i");
  check(config.i, CheckMode::GT, 0, "i");
}

struct Derived : virtual public Base {
  std::vector<int> vec = {1, 2, 3};
};

void declare_config(Derived& config) {
  using namespace config;
  name("Derived");
  base<Base>(config);
  field(config.vec, "vec");
  check(config.vec.size(), CheckMode::EQ, size_t(3), "vec.size()");
}

struct SequentialDerived : public Derived {
  std::string s = "Some text";
};

void declare_config(SequentialDerived& config) {
  using namespace config;
  name("SequentialDerived");
  base<Derived>(config);
  field(config.s, "s");
  checkCondition(!config.s.empty(), "param 's' may not be empty.");
}

struct OtherBase {
  float f = 1.23;
};

void declare_config(OtherBase& config) {
  using namespace config;
  name("OtherBase");
  field(config.f, "f");
  check(config.f, CheckMode::GT, 1.f, "f");
}

struct MultipleDerived : virtual public Base, public OtherBase {
  double d = 5.67;
};

void declare_config(MultipleDerived& config) {
  using namespace config;
  name("MultipleDerived");
  base<Base>(config);
  base<OtherBase>(config);
  field(config.d, "d");
  check(config.d, CheckMode::GT, 0.0, "d");
}

struct DiamondDerived : public Derived, public MultipleDerived {
  std::map<int, int> map = {{1, 1}, {2, 2}};
};

void declare_config(DiamondDerived& config) {
  using namespace config;
  name("DiamondDerived");
  base<Derived>(config);
  base<MultipleDerived>(config);
  field(config.map, "map");
  check(config.map.size(), CheckMode::GE, 2, "map.size()");
}

YAML::Node defaultValues() {
  YAML::Node data;
  data["i"] = 1;
  data["vec"] = std::vector<int>({1, 2, 3});
  data["s"] = "Some text";
  data["f"] = 1.23f;
  data["d"] = 5.67;
  data["map"] = std::map<int, int>({{1, 1}, {2, 2}});
  return data;
}

YAML::Node modifiedValues() {
  YAML::Node data;
  data["i"] = 2;
  data["vec"] = std::vector<int>({4, 5, 6});
  data["s"] = "Some text different text";
  data["f"] = 4.56f;
  data["d"] = 89.012;
  data["map"] = std::map<int, int>({{2, 2}, {3, 3}, {4, 4}});
  return data;
}

YAML::Node invalidValues() {
  YAML::Node data;
  data["i"] = -12;
  data["vec"] = std::vector<int>({5, 6, 7, 8});
  data["s"] = "";
  data["f"] = 0.123f;
  data["d"] = -89;
  data["map"] = std::map<int, int>({{5, 5}});
  return data;
}

// TODO(lschmid): One could also add test for whether identical fields (including namespace) are got/set multiple times.
// However, this is more a user mistake and could cause a lot of unnecessary warnings if intended.

TEST(Inheritance, sequentialInheritanceGet) {
  const SequentialDerived config;
  const internal::MetaData data = internal::Visitor::getValues(config);
  const YAML::Node default_values = defaultValues();
  EXPECT_EQ(data.name, "SequentialDerived");
  EXPECT_EQ(data.field_infos.size(), 3ul);
  expectEqual(data.data["i"], default_values["i"]);
  expectEqual(data.data["vec"], default_values["vec"]);
  expectEqual(data.data["s"], default_values["s"]);
}

TEST(Inheritance, sequentialInheritanceSet) {
  SequentialDerived config;
  internal::Visitor::setValues(config, modifiedValues());
  EXPECT_EQ(config.i, 2);
  EXPECT_EQ(config.vec, std::vector<int>({4, 5, 6}));
  EXPECT_EQ(config.s, "Some text different text");
}

TEST(Inheritance, sequentialInheritanceCheck) {
  SequentialDerived config;
  internal::Visitor::setValues(config, invalidValues());
  const internal::MetaData data = internal::Visitor::getChecks(config);
  EXPECT_EQ(data.checks.size(), 3);
  for (const auto& check : data.checks) {
    EXPECT_FALSE(check->valid()) << toString(config);
  }
}

TEST(Inheritance, multipleInheritanceGet) {
  const MultipleDerived config;
  const internal::MetaData data = internal::Visitor::getValues(config);
  const YAML::Node default_values = defaultValues();
  EXPECT_EQ(data.name, "MultipleDerived");
  EXPECT_EQ(data.field_infos.size(), 3ul);
  expectEqual(data.data["i"], default_values["i"]);
  expectEqual(data.data["f"], default_values["f"]);
  expectEqual(data.data["d"], default_values["d"]);
}

TEST(Inheritance, multipleInheritanceSet) {
  MultipleDerived config;
  internal::Visitor::setValues(config, modifiedValues());
  EXPECT_EQ(config.i, 2);
  EXPECT_EQ(config.f, 4.56f);
  EXPECT_EQ(config.d, 89.012);
}

TEST(Inheritance, multipleInheritanceCheck) {
  MultipleDerived config;
  internal::Visitor::setValues(config, invalidValues());
  const internal::MetaData data = internal::Visitor::getChecks(config);
  EXPECT_EQ(data.checks.size(), 3);
  for (const auto& check : data.checks) {
    EXPECT_FALSE(check->valid());
  }
}

TEST(Inheritance, diamondInheritanceGet) {
  const DiamondDerived config;
  const internal::MetaData data = internal::Visitor::getValues(config);
  const YAML::Node default_values = defaultValues();
  EXPECT_EQ(data.name, "DiamondDerived");
  EXPECT_EQ(data.field_infos.size(), 5ul);
  expectEqual(data.data["i"], default_values["i"]);
  expectEqual(data.data["f"], default_values["f"]);
  expectEqual(data.data["vec"], default_values["vec"]);
  expectEqual(data.data["d"], default_values["d"]);
  expectEqual(data.data["map"], default_values["map"]);
}

TEST(Inheritance, diamondInheritanceSet) {
  DiamondDerived config;
  internal::Visitor::setValues(config, modifiedValues());
  EXPECT_EQ(config.i, 2);
  EXPECT_EQ(config.f, 4.56f);
  EXPECT_EQ(config.vec, std::vector<int>({4, 5, 6}));
  EXPECT_EQ(config.d, 89.012);
  const std::map<int, int> map({{2, 2}, {3, 3}, {4, 4}});
  EXPECT_EQ(config.map, map);
}

TEST(Inheritance, diamondInheritanceCheck) {
  DiamondDerived config;
  internal::Visitor::setValues(config, invalidValues());
  const internal::MetaData data = internal::Visitor::getChecks(config);
  EXPECT_EQ(data.checks.size(), 5);
  for (const auto& check : data.checks) {
    EXPECT_FALSE(check->valid());
  }
}

}  // namespace config::test
