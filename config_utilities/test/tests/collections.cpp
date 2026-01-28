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

#include "config_utilities/types/collections.h"

#include <list>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"

namespace config::test {

struct PlusOneConversion {
  static int toIntermediate(int value, std::string&) { return value + 1; }
  static void fromIntermediate(int intermediate, int& value, std::string&) { value = intermediate - 1; }
};

struct MinusOneConversion {
  static int toIntermediate(int value, std::string&) { return value - 1; }
  static void fromIntermediate(int intermediate, int& value, std::string&) { value = intermediate + 1; }
};

struct KeyConversionStruct {
  std::map<int, std::string> key_map;
  std::unordered_map<int, std::string> unordered_key_map;

  std::map<int, std::string> ordered_key_map() const {
    return std::map<int, std::string>(unordered_key_map.begin(), unordered_key_map.end());
  }
};

void declare_config(KeyConversionStruct& config) {
  field<MapKeyConverter<PlusOneConversion>>(config.key_map, "key_map");
  field<MapKeyConverter<PlusOneConversion>>(config.unordered_key_map, "key_map");
}

struct ValueConversionStruct {
  std::map<std::string, int> value_map;
  std::unordered_map<std::string, int> unordered_value_map;

  std::map<std::string, int> ordered_value_map() const {
    return std::map<std::string, int>(unordered_value_map.begin(), unordered_value_map.end());
  }
};

void declare_config(ValueConversionStruct& config) {
  field<MapValueConverter<PlusOneConversion>>(config.value_map, "value_map");
  field<MapValueConverter<PlusOneConversion>>(config.unordered_value_map, "value_map");
}

struct KeyValueConversionStruct {
  std::map<int, int> int_map;
  std::unordered_map<int, int> unordered_int_map;

  std::map<int, int> ordered_int_map() const {
    return std::map<int, int>(unordered_int_map.begin(), unordered_int_map.end());
  }
};

void declare_config(KeyValueConversionStruct& config) {
  using FullConversion = MapKeyValueConverter<PlusOneConversion, MinusOneConversion>;
  field<FullConversion>(config.int_map, "int_map");
  field<FullConversion>(config.unordered_int_map, "int_map");
}

struct SeqConversionStruct {
  std::list<int> value_list;
  std::vector<int> value_vec;

  std::list<int> vec_as_list() const { return std::list<int>(value_vec.begin(), value_vec.end()); }
};

void declare_config(SeqConversionStruct& config) {
  field<SequenceConverter<PlusOneConversion>>(config.value_list, "values");
  field<SequenceConverter<PlusOneConversion>>(config.value_vec, "values");
}

TEST(Collections, ConvertKeys) {
  auto node = YAML::Load(R"yaml(
key_map:
  2: world
  3: hello
  0: test
)yaml");
  auto result = fromYaml<KeyConversionStruct>(node);
  std::map<int, std::string> expected{{-1, "test"}, {1, "world"}, {2, "hello"}};
  EXPECT_EQ(expected, result.key_map);
  EXPECT_EQ(expected, result.ordered_key_map());

  // make sure second parse overrides first
  node = YAML::Load(R"yaml(
key_map:
  0: foo
  )yaml");
  EXPECT_TRUE(updateFromYaml(result, node));
  expected = {{-1, "foo"}};
  EXPECT_EQ(expected, result.key_map);
  EXPECT_EQ(expected, result.ordered_key_map());
}

TEST(Collections, ConvertValues) {
  auto node = YAML::Load(R"yaml(
value_map:
  world: 2
  hello: 3
  test: 0
)yaml");
  auto result = fromYaml<ValueConversionStruct>(node);
  std::map<std::string, int> expected{{"test", -1}, {"world", 1}, {"hello", 2}};
  EXPECT_EQ(expected, result.value_map);
  EXPECT_EQ(expected, result.ordered_value_map());

  // make sure second parse overrides first
  node = YAML::Load(R"yaml(
value_map:
  foo: 0
  )yaml");
  EXPECT_TRUE(updateFromYaml(result, node));
  expected = {{"foo", -1}};
  EXPECT_EQ(expected, result.value_map);
  EXPECT_EQ(expected, result.ordered_value_map());
}

TEST(Collections, ConvertFullMap) {
  auto node = YAML::Load(R"yaml(int_map: {1: 2, 2: 3, 3: 0})yaml");
  auto result = fromYaml<KeyValueConversionStruct>(node);
  std::map<int, int> expected{{0, 3}, {1, 4}, {2, 1}};
  EXPECT_EQ(expected, result.int_map);
  EXPECT_EQ(expected, result.ordered_int_map());

  // make sure second parse overrides first
  node = YAML::Load(R"yaml(int_map: {2: -2})yaml");
  EXPECT_TRUE(updateFromYaml(result, node));
  expected = {{1, -1}};
  EXPECT_EQ(expected, result.int_map);
  EXPECT_EQ(expected, result.ordered_int_map());
}

TEST(Collections, ConvertSequence) {
  auto node = YAML::Load(R"yaml(values: [1, 2, 3])yaml");
  auto result = fromYaml<SeqConversionStruct>(node);
  std::list<int> expected{0, 1, 2};
  EXPECT_EQ(expected, result.value_list);
  EXPECT_EQ(expected, result.vec_as_list());

  // make sure second parse overrides first
  node = YAML::Load(R"yaml(values: [0, 2, 4, 6])yaml");
  EXPECT_TRUE(updateFromYaml(result, node));
  expected = {-1, 1, 3, 5};
  EXPECT_EQ(expected, result.value_list);
  EXPECT_EQ(expected, result.vec_as_list());
}

}  // namespace config::test
