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

#include "config_utilities/test/default_config.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/types/enum.h"

namespace config::test {

void declare_config(SubSubConfig& config) {
  name("SubSubConfig");
  field(config.i, "i");
  check(config.i, CheckMode::GT, 0, "i");
}

void declare_config(SubConfig& config) {
  name("SubConfig");
  field(config.i, "i");
  enter_namespace("nested_ns");
  field(config.sub_sub_config, "sub_sub_config", false);
  check(config.i, CheckMode::GT, 0, "i");
}

void declare_config(DefaultConfig& config) {
  name("DefaultConfig");
  field(config.i, "i", "m");
  field(config.f, "f", "s");
  field(config.d, "d", "m/s");
  field(config.b, "b");
  field(config.u8, "u8");
  field(config.s, "s");
  field(config.vec, "vec", "frames");
  field(config.map, "map");
  field(config.set, "set");
  field(config.mat, "mat");
  enum_field(config.my_enum, "my_enum", {"A", "B", "C"});
  enum_field(config.my_strange_enum,
             "my_strange_enum",
             {{DefaultConfig::StrangeEnum::kX, "X"},
              {DefaultConfig::StrangeEnum::kY, "Y"},
              {DefaultConfig::StrangeEnum::kZ, "Z"}});
  enter_namespace("sub_ns");
  field(config.sub_config, "sub_config", false);
  switch_namespace("sub_sub_ns");
  field(config.sub_sub_config, "sub_sub_config", false);

  check(config.i, CheckMode::GT, 0, "i");
  check(config.f, CheckMode::GE, 0.f, "f");
  check(config.d, CheckMode::LT, 4.0, "d");
  check(config.u8, CheckMode::LE, uint8_t(5), "u8");
  check(config.s, CheckMode::EQ, std::string("test string"), "s");
  check(config.b, CheckMode::NE, false, "b");
  checkCondition(config.vec.size() == 3, "param 'vec' must b of size '3'");
  checkInRange(config.d, 0.0, 500.0, "d");
}

YAML::Node DefaultConfig::defaultValues() {
  YAML::Node data;
  data["i"] = 1;
  data["f"] = 2.1f;
  data["d"] = 3.2;
  data["b"] = true;
  data["u8"] = 4;
  data["s"] = "test string";
  data["vec"] = std::vector<int>({1, 2, 3});
  const std::map<std::string, int> map({{"a", 1}, {"b", 2}, {"c", 3}});
  data["map"] = map;
  data["set"] = std::vector<float>({1.1f, 2.2, 3.3f});
  data["mat"].push_back(std::vector<double>({1, 0, 0}));
  data["mat"].push_back(std::vector<double>({0, 1, 0}));
  data["mat"].push_back(std::vector<double>({0, 0, 1}));
  data["my_enum"] = "A";
  data["my_strange_enum"] = "X";
  data["sub_ns"]["i"] = 1;
  data["sub_ns"]["nested_ns"]["i"] = 1;
  data["sub_sub_ns"]["i"] = 1;
  return data;
}

YAML::Node DefaultConfig::modifiedValues() {
  YAML::Node data;
  data["i"] = 2;
  data["f"] = -1.f;
  data["d"] = 3.14159; // intentionally avoid precision issues
  data["b"] = false;
  data["u8"] = 255;
  data["s"] = "a different test string";
  data["vec"] = std::vector<int>({2, 3, 4, 5});
  const std::map<std::string, int> map({{"x", 24}, {"y", 25}, {"z", 26}});
  data["map"] = map;
  data["set"] = std::vector<float>({11.11, 22.22, 33.33, 44.44});
  data["mat"].push_back(std::vector<double>({1, 2, 3}));
  data["mat"].push_back(std::vector<double>({4, 5, 6}));
  data["mat"].push_back(std::vector<double>({7, 8, 9}));
  data["my_enum"] = "B";
  data["my_strange_enum"] = "Z";
  data["sub_ns"]["i"] = 2;
  data["sub_ns"]["nested_ns"]["i"] = 3;
  data["sub_sub_ns"]["i"] = 4;
  return data;
}

void DefaultConfig::expectDefaultValues() {
  EXPECT_EQ(i, 1);
  EXPECT_EQ(f, 2.1f);
  EXPECT_EQ(d, 3.2);
  EXPECT_EQ(b, true);
  EXPECT_EQ(u8, 4);
  EXPECT_EQ(s, "test string");
  EXPECT_EQ(vec, std::vector<int>({1, 2, 3}));
  const std::map<std::string, int> map({{"a", 1}, {"b", 2}, {"c", 3}});
  EXPECT_EQ(map, map);
  EXPECT_EQ(set, std::set<float>({1.1f, 2.2, 3.3f}));
  const auto mat = Eigen::Matrix<double, 3, 3>::Identity();
  EXPECT_EQ(mat, mat);
  EXPECT_EQ(my_enum, DefaultConfig::Enum::kA);
  EXPECT_EQ(my_strange_enum, DefaultConfig::StrangeEnum::kX);
  EXPECT_EQ(sub_config.i, 1);
  EXPECT_EQ(sub_config.sub_sub_config.i, 1);
  EXPECT_EQ(sub_sub_config.i, 1);
}

void DefaultConfig::expectModifiedValues() {
  EXPECT_EQ(i, 2);
  EXPECT_EQ(f, -1.f);
  EXPECT_EQ(d, 3.14159);
  EXPECT_EQ(b, false);
  EXPECT_EQ(u8, 255);
  EXPECT_EQ(s, "a different test string");
  EXPECT_EQ(vec, std::vector<int>({2, 3, 4, 5}));
  const std::map<std::string, int> map({{"x", 24}, {"y", 25}, {"z", 26}});
  EXPECT_EQ(map, map);
  EXPECT_EQ(set, std::set<float>({11.11, 22.22, 33.33, 44.44}));
  Eigen::Matrix3d mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  EXPECT_EQ(mat, mat);
  EXPECT_EQ(my_enum, DefaultConfig::Enum::kB);
  EXPECT_EQ(my_strange_enum, DefaultConfig::StrangeEnum::kZ);
  EXPECT_EQ(sub_config.i, 2);
  EXPECT_EQ(sub_config.sub_sub_config.i, 3);
  EXPECT_EQ(sub_sub_config.i, 4);
}

}  // namespace config::test
