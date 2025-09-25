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

#include <sstream>

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"

namespace config::test {

TEST(YamlParsing, parsefromYaml) {
  DefaultConfig config;
  YAML::Node data = DefaultConfig::modifiedValues();
  std::string error;

  auto val1 = internal::YamlParser::fromYaml<int>(data["i"]);
  EXPECT_TRUE(val1.has_value());
  EXPECT_EQ(*val1, 2);

  auto val2 = internal::YamlParser::fromYaml<float>(data["f"]);
  EXPECT_TRUE(val2.has_value());
  EXPECT_EQ(*val2, -1.f);

  auto val3 = internal::YamlParser::fromYaml<double>(data["d"]);
  EXPECT_TRUE(val3.has_value());
  EXPECT_EQ(*val3, 3.14159);

  auto val4 = internal::YamlParser::fromYaml<bool>(data["b"]);
  EXPECT_TRUE(val4.has_value());
  EXPECT_EQ(*val4, false);

  auto val5 = internal::YamlParser::fromYaml<uint8_t>(data["u8"]);
  EXPECT_TRUE(val5.has_value());
  EXPECT_EQ(*val5, 255);

  auto val6 = internal::YamlParser::fromYaml<std::string>(data["s"]);
  EXPECT_TRUE(val6.has_value());
  EXPECT_EQ(*val6, "a different test string");

  auto val7 = internal::YamlParser::fromYaml<std::vector<int>>(data["vec"]);
  EXPECT_TRUE(val7.has_value());
  EXPECT_EQ(*val7, std::vector<int>({2, 3, 4, 5}));

  auto val8 = internal::YamlParser::fromYaml<std::map<std::string, int>>(data["map"]);
  EXPECT_TRUE(val8.has_value());
  const std::map<std::string, int> map({{"x", 24}, {"y", 25}, {"z", 26}});
  EXPECT_EQ(*val8, map);

  auto val9 = internal::YamlParser::fromYaml<std::set<float>>(data["set"]);
  EXPECT_TRUE(val9.has_value());
  EXPECT_EQ(*val9, std::set<float>({11.11, 22.22, 33.33, 44.44}));

  auto val10 = internal::YamlParser::fromYaml<Eigen::Matrix3d>(data["mat"]);
  EXPECT_TRUE(val10.has_value());
  Eigen::Matrix3d mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  EXPECT_EQ(*val10, mat);
}

TEST(YamlParsing, conversionFailure) {
  DefaultConfig config;
  YAML::Node node = YAML::Load(R"yaml(
vec: "Invalid Value to create a vector from"
map: "Invalid Value to create a map from"
mat:
  - [1, 2, 3]
  - [4, 5, 6]
  - [7, 8, 9, 10]
i: "Value that can't be cast to int"
my_enum: "D"
sub_ns:
  i: "Value that can't be cast to int"
  nested_ns:
    i: "Value that can't be cast to int"
  )yaml");
  internal::MetaData data = internal::Visitor::setValues(config, node);
  std::vector<std::unique_ptr<internal::Warning>> errors;
  data.performOnAll([&errors](const internal::MetaData& d) {
    for (const auto& error : d.errors) {
      errors.emplace_back(error->clone());
    }
  });
  EXPECT_EQ(data.errors.size(), 5ul);
  ASSERT_EQ(errors.size(), 7ul);
  EXPECT_EQ(errors[0]->name(), "i");
  EXPECT_EQ(errors[1]->name(), "vec");
  EXPECT_EQ(errors[2]->name(), "map");
  EXPECT_EQ(errors[3]->name(), "mat");
  EXPECT_EQ(errors[4]->name(), "my_enum");
  EXPECT_EQ(errors[5]->name(), "i");
  EXPECT_EQ(errors[6]->name(), "i");
  EXPECT_EQ(errors[1]->message(), "Data is not a sequence");
  EXPECT_EQ(errors[2]->message(), "Data is not a map");
  EXPECT_EQ(errors[4]->message(), "Name 'D' is out of bounds for enum with names ['A', 'B', 'C']");
}

TEST(YamlParsing, overflowConversionFailure) {
  const auto node = YAML::Load(R"yaml({under: -1, over: 256})yaml");

  {  // values below [0, 255] cause errors
    uint8_t value = 0;
    std::string error;
    auto val = internal::YamlParser::fromYaml<uint8_t>(node["under"], &error);
    EXPECT_FALSE(val.has_value());
    EXPECT_EQ(value, 0u);
    EXPECT_EQ(error, "Value '-1' underflows storage min of '0'.");
  }

  {  // values above [0, 255] cause errors
    uint8_t value = 0;
    std::string error;
    auto val = internal::YamlParser::fromYaml<uint8_t>(node["over"], &error);
    EXPECT_FALSE(val.has_value());
    EXPECT_EQ(value, 0u);
    EXPECT_EQ(error, "Value '256' overflows storage max of '255'.");
  }
}

TEST(YamlParsing, setValues) {
  YAML::Node data = DefaultConfig::modifiedValues();
  DefaultConfig config;
  internal::MetaData meta_data = internal::Visitor::setValues(config, data);
  config.expectModifiedValues();
  EXPECT_FALSE(meta_data.hasErrors());

  // Make sure the input node is not modified.
  expectEqual(data, DefaultConfig::modifiedValues());

  // Test that the config is not modified if no data is found is empty.
  data = YAML::Node();
  meta_data = internal::Visitor::setValues(config, data);
  config.expectModifiedValues();
  EXPECT_FALSE(meta_data.hasErrors());

  data["i"] = "Not an int";
  data["u8"] = -1;
  data["d"] = static_cast<long double>(std::numeric_limits<double>::max()) * 2;
  data["vec"] = std::map<std::string, int>({{"1_str", 1}, {"2_str", 2}});
  data["map"] = std::vector<int>({1, 2, 3});
  data["set"] = std::vector<float>({11.11, 22.22, 33.33, 44.44, 11.11});
  data["mat"].push_back(std::vector<int>({1, 2, 3}));
  data["mat"].push_back(std::vector<int>({1, 2, 3, 4}));
  data["mat"].push_back(std::vector<int>({1, 2, 3}));
  data["my_enum"] = "OutOfList";
  meta_data = internal::Visitor::setValues(config, data);
  config.expectModifiedValues();
  EXPECT_TRUE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 8ul);
}

TEST(YamlParsing, getValues) {
  DefaultConfig config;
  internal::MetaData meta_data = internal::Visitor::getValues(config);

  config.expectDefaultValues();
  expectEqual(meta_data.data, DefaultConfig::defaultValues());
  EXPECT_FALSE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 0ul);
  meta_data.performOnAll([](const internal::MetaData& d) {
    for (const auto& field : d.field_infos) {
      EXPECT_TRUE(field.isDefault());
    }
  });
  EXPECT_EQ(meta_data.name, "DefaultConfig");

  internal::Visitor::setValues(config, DefaultConfig::modifiedValues());
  meta_data = internal::Visitor::getValues(config);
  config.expectModifiedValues();
  expectEqual(meta_data.data, DefaultConfig::modifiedValues());
  EXPECT_FALSE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 0ul);
  meta_data.performOnAll([](const internal::MetaData& d) {
    for (const auto& field : d.field_infos) {
      EXPECT_FALSE(field.isDefault());
    }
  });
}

TEST(YamlParsing, configFromYaml) {
  const YAML::Node data = DefaultConfig::modifiedValues();
  auto config = fromYaml<DefaultConfig>(data);
  config.expectModifiedValues();

  // Make sure the input node is not modified.
  expectEqual(data, DefaultConfig::modifiedValues());
}

TEST(YamlParsing, configToYAML) {
  DefaultConfig config;
  expectEqual(toYaml(config), DefaultConfig::defaultValues());

  config = fromYaml<DefaultConfig>(DefaultConfig::modifiedValues());
  expectEqual(toYaml(config), DefaultConfig::modifiedValues());
}

struct EmptyConfig {
  std::map<std::string, double> empty_map;
  std::set<int64_t> empty_set;
  std::vector<std::string> empty_vector;
  std::list<float> empty_list;
};

void declare_config(EmptyConfig& conf) {
  name("EmptyConfig");
  field(conf.empty_map, "empty_map");
  field(conf.empty_set, "empty_set");
  field(conf.empty_vector, "empty_vector");
  field(conf.empty_list, "empty_list");
}

TEST(YamlParsing, emptyCollections) {
  EmptyConfig empty_config;
  YAML::Emitter out;
  out << YAML::Flow << toYaml(empty_config);
  const std::string result = out.c_str();
  const std::string expected = "{empty_map: {}, empty_set: [], empty_vector: [], empty_list: []}";
  EXPECT_EQ(expected, result);
}

TEST(YamlParsing, updateCorrect) {
  SubSubConfig config;
  // configs that don't pass checks should fail to update underlying config
  EXPECT_FALSE(updateFromYaml(config, YAML::Load("i: -1")));
  EXPECT_EQ(config.i, 1);
  // configs that 't pass checks should update underlying config
  EXPECT_TRUE(updateFromYaml(config, YAML::Load("i: 5")));
  EXPECT_EQ(config.i, 5);
}

}  // namespace config::test
