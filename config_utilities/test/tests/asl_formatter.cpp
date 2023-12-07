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
#include "config_utilities/formatting/asl.h"
#include "config_utilities/internal/meta_data.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"

namespace config::test {

int countLines(const std::string& str) {
  int count = 1;
  for (char c : str) {
    if (c == '\n') {
      ++count;
    }
  }
  return count;
}

struct TestConfig : public DefaultConfig {
  std::string test1 = "Short Value";
  std::string test2 = "A really really really ridiculously long string that will be wrapped.";
  std::string test3 = "A really really really ridiculously long string that will also be wrapped.";
};

void declare_config(TestConfig& config) {
  name("Test Config");
  base<DefaultConfig>(config);
  config::field(config.test1, "A ridiculously long field name that will not be wrapped", "ms");
  config::field(config.test2, "A ridiculously long field name that will also not be wrapped", "custom unit");
  config::field(config.test3,
                "A really really really really really really ridiculously long field name that will be wrapped",
                "and has a long unit");
}

struct ArrayConfig {
  float f = 0.f;
};

void declare_config(ArrayConfig& config) {
  name("ArrayConfig");
  field(config.f, "f");
  check(config.f, GE, 0, "f");
}

struct ConfigUsingArrays {
  std::vector<ArrayConfig> arr;
};

void declare_config(ConfigUsingArrays& config) {
  name("ConfigUsingArrays");
  field(config.arr, "arr");
}

TEST(AslFormatter, DataToString) {
  YAML::Node data = internal::Visitor::getValues(TestConfig()).data;
  // note: float reformatting should have no effect on other fields (and fixes full precision formatting for internal
  // yaml representation)
  EXPECT_EQ(internal::dataToString(data["i"], true), "1");
  EXPECT_EQ(internal::dataToString(data["f"], true), "2.1");
  EXPECT_EQ(internal::dataToString(data["d"], true), "3.2");
  EXPECT_EQ(internal::dataToString(data["b"], true), "true");
  EXPECT_EQ(internal::dataToString(data["u8"], true), "4");
  EXPECT_EQ(internal::dataToString(data["s"], true), "test string");
  EXPECT_EQ(internal::dataToString(data["vec"], true), "[1, 2, 3]");
  EXPECT_EQ(internal::dataToString(data["map"], true), "{a: 1, b: 2, c: 3}");
  EXPECT_EQ(internal::dataToString(data["set"], true), "[1.1, 2.2, 3.3]");
  EXPECT_EQ(internal::dataToString(data["mat"], true), "[[1, 0, 0], [0, 1, 0], [0, 0, 1]]");
  YAML::Node nested_set;
  nested_set["a"]["x"] = 1;
  nested_set["a"]["y"] = 2;
  nested_set["b"]["x"] = 3;
  nested_set["b"]["y"] = 4;
  EXPECT_EQ(internal::dataToString(nested_set, true), "{a: {x: 1, y: 2}, b: {x: 3, y: 4}}");
}

TEST(AslFormatter, FormatErrors) {
  internal::MetaData data;
  data.name = "Config 1";
  data.errors.emplace_back(new internal::Warning("Field 1", "Error 1"));
  data.errors.emplace_back(new internal::Warning("", "Error 2"));
  internal::MetaData& d = data.sub_configs.emplace_back();
  d.name = "Config 2";
  d.errors.emplace_back(new internal::Warning("Field 3", "Error 3"));
  d.errors.emplace_back(new internal::Warning("Field 4", "Error 4"));
  internal::MetaData& d2 = data.sub_configs.emplace_back();
  internal::MetaData& d3 = d2.sub_configs.emplace_back();
  d3.name = "Config 3";
  d3.errors.emplace_back(new internal::Warning("", "Error 5"));
  internal::MetaData& d4 = d3.sub_configs.emplace_back();
  d4.name = "Config 4";
  d4.errors.emplace_back(new internal::Warning("Field 6", "Error 6"));

  std::string formatted = internal::Formatter::formatErrors(data);
  EXPECT_EQ(countLines(formatted), 9);

  std::string expected = R"""( 'Config 1':
=================================== Config 1 ===================================
Warning: Failed to parse param 'Field 1': Error 1.
Warning: Failed to parse param: Error 2.
Warning: Failed to parse param 'Field 3': Error 3.
Warning: Failed to parse param 'Field 4': Error 4.
Warning: Failed to parse param: Error 5.
Warning: Failed to parse param 'Field 6': Error 6.
================================================================================)""";
  EXPECT_EQ(formatted, expected);

  Settings().inline_subconfig_field_names = false;
  formatted = internal::Formatter::formatErrors(data);
  EXPECT_EQ(countLines(formatted), 12);

  expected = R"""( 'Config 1':
=================================== Config 1 ===================================
Warning: Failed to parse param 'Field 1': Error 1.
Warning: Failed to parse param: Error 2.
----------------------------------- Config 2 -----------------------------------
Warning: Failed to parse param 'Field 3': Error 3.
Warning: Failed to parse param 'Field 4': Error 4.
----------------------------------- Config 3 -----------------------------------
Warning: Failed to parse param: Error 5.
----------------------------------- Config 4 -----------------------------------
Warning: Failed to parse param 'Field 6': Error 6.
================================================================================)""";
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, FormatChecks) {
  DefaultConfig config;
  config.i = -1;
  config.f = -1.f;
  config.d = 100.0;
  // TODO(lschmid): u8 as int formatting is currently not supported in the checks. Maybe using the field declaration and
  // yaml parser could potentially resolve this. config.u8 = 26;
  config.s = "";
  config.vec = {1, 2};
  config.b = false;
  config.d = 1000.0;
  config.sub_config.i = -1;
  config.sub_sub_config.i = -1;
  config.sub_config.sub_sub_config.i = -1;

  Settings().restoreDefaults();
  Settings().inline_subconfig_field_names = false;
  internal::MetaData data = internal::Visitor::getChecks(config);
  std::string formatted = internal::Formatter::formatErrors(data);
  std::string expected = R"""( 'DefaultConfig':
================================ DefaultConfig =================================
Warning: Check [1/8] failed for 'i': param > 0 (is: '-1').
Warning: Check [2/8] failed for 'f': param >= 0 (is: '-1').
Warning: Check [3/8] failed for 'd': param < 4 (is: '1000').
Warning: Check [5/8] failed for 's': param == test string (is: '').
Warning: Check [6/8] failed for 'b': param != 0 (is: '0').
Warning: Check [7/8] failed: param 'vec' must b of size '3'.
Warning: Check [8/8] failed for 'd': param within [0, 500] (is: '1000').
---------------------------------- SubConfig -----------------------------------
Warning: Check [1/1] failed for 'i': param > 0 (is: '-1').
--------------------------------- SubSubConfig ---------------------------------
Warning: Check [1/1] failed for 'i': param > 0 (is: '-1').
--------------------------------- SubSubConfig ---------------------------------
Warning: Check [1/1] failed for 'i': param > 0 (is: '-1').
================================================================================
  )""";

  Settings().inline_subconfig_field_names = true;
  data = internal::Visitor::getChecks(config);
  formatted = internal::Formatter::formatErrors(data);
  expected = R"""( 'DefaultConfig':
================================ DefaultConfig =================================
Warning: Check [1/11] failed for 'i': param > 0 (is: '-1').
Warning: Check [2/11] failed for 'f': param >= 0 (is: '-1').
Warning: Check [3/11] failed for 'd': param < 4 (is: '1000').
Warning: Check [5/11] failed for 's': param == test string (is: '').
Warning: Check [6/11] failed for 'b': param != 0 (is: '0').
Warning: Check [7/11] failed: param 'vec' must b of size '3'.
Warning: Check [8/11] failed for 'd': param within [0, 500] (is: '1000').
Warning: Check [9/11] failed for 'sub_config.i': param > 0 (is: '-1').
Warning: Check [10/11] failed for 'sub_config.sub_sub_config.i': param > 0 (is:
         '-1').
Warning: Check [11/11] failed for 'sub_sub_config.i': param > 0 (is: '-1').
================================================================================)""";
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, FormatConfig) {
  internal::MetaData data = internal::Visitor::getValues(TestConfig());

  Settings().indicate_default_values = false;
  Settings().indicate_units = false;
  Settings().inline_subconfig_field_names = true;
  Settings().reformat_floats = true;
  std::string formatted = internal::Formatter::formatConfig(data);
  std::string expected =
      R"""(================================= Test Config ==================================
i:                            1
f:                            2.1
d:                            3.2
b:                            true
u8:                           4
s:                            test string
vec:                          [1, 2, 3]
map:                          {a: 1, b: 2, c: 3}
set:                          [1.1, 2.2, 3.3]
mat:                          [[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]]
my_enum:                      A
my_strange_enum:              X
A ridiculously long field name that will not be wrapped: Short Value
A ridiculously long field name that will also not be wrapped:
                              A really really really ridiculously long string th
                              at will be wrapped.
A really really really really really really ridiculously long field name that wi
ll be wrapped:                A really really really ridiculously long string th
                              at will also be wrapped.
sub_config [SubConfig]:
   i:                         1
   sub_sub_config [SubSubConfig]:
      i:                      1
sub_sub_config [SubSubConfig]:
   i:                         1
================================================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);

  Settings().print_width = 50;
  formatted = internal::Formatter::formatConfig(data);
  expected =
      R"""(================== Test Config ===================
i:                            1
f:                            2.1
d:                            3.2
b:                            true
u8:                           4
s:                            test string
vec:                          [1, 2, 3]
map:                          {a: 1, b: 2, c: 3}
set:                          [1.1, 2.2, 3.3]
mat:                          [[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]]
my_enum:                      A
my_strange_enum:              X
A ridiculously long field name that will not be wr
apped:                        Short Value
A ridiculously long field name that will also not
be wrapped:                   A really really real
                              ly ridiculously long
                              string that will be
                              wrapped.
A really really really really really really ridicu
lously long field name that will be wrapped:
                              A really really real
                              ly ridiculously long
                              string that will als
                              o be wrapped.
sub_config [SubConfig]:
   i:                         1
   sub_sub_config [SubSubConfig]:
      i:                      1
sub_sub_config [SubSubConfig]:
   i:                         1
==================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);

  Settings().print_width = 80;
  Settings().print_indent = 20;
  formatted = internal::Formatter::formatConfig(data);
  expected =
      R"""(================================= Test Config ==================================
i:                  1
f:                  2.1
d:                  3.2
b:                  true
u8:                 4
s:                  test string
vec:                [1, 2, 3]
map:                {a: 1, b: 2, c: 3}
set:                [1.1, 2.2, 3.3]
mat:                [[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]]
my_enum:            A
my_strange_enum:    X
A ridiculously long field name that will not be wrapped: Short Value
A ridiculously long field name that will also not be wrapped:
                    A really really really ridiculously long string that will be
                    wrapped.
A really really really really really really ridiculously long field name that wi
ll be wrapped:      A really really really ridiculously long string that will al
                    so be wrapped.
sub_config [SubConfig]:
   i:               1
   sub_sub_config [SubSubConfig]:
      i:            1
sub_sub_config [SubSubConfig]:
   i:               1
================================================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, FormatUnits) {
  Settings().indicate_default_values = false;
  Settings().indicate_units = true;
  Settings().inline_subconfig_field_names = true;
  Settings().print_width = 80;  // force print width to be consistent for tests
  Settings().print_indent = 20;

  internal::MetaData data = internal::Visitor::getValues(TestConfig());
  const std::string formatted = internal::Formatter::formatConfig(data);
  const std::string expected =
      R"""(================================= Test Config ==================================
i [m]:              1
f [s]:              2.1
d [m/s]:            3.2
b:                  true
u8:                 4
s:                  test string
vec [frames]:       [1, 2, 3]
map:                {a: 1, b: 2, c: 3}
set:                [1.1, 2.2, 3.3]
mat:                [[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]]
my_enum:            A
my_strange_enum:    X
A ridiculously long field name that will not be wrapped [ms]: Short Value
A ridiculously long field name that will also not be wrapped [custom unit]:
                    A really really really ridiculously long string that will be
                    wrapped.
A really really really really really really ridiculously long field name that wi
ll be wrapped [and has a long unit]:
                    A really really really ridiculously long string that will al
                    so be wrapped.
sub_config [SubConfig]:
   i:               1
   sub_sub_config [SubSubConfig]:
      i:            1
sub_sub_config [SubSubConfig]:
   i:               1
================================================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, FormatDefaultValues) {
  Settings().indicate_default_values = true;
  Settings().indicate_units = false;
  Settings().inline_subconfig_field_names = true;
  Settings().print_indent = 20;

  const internal::MetaData default_data = internal::Visitor::getValues(TestConfig());
  std::string formatted = internal::Formatter::formatConfig(default_data);
  std::string expected = R"""(================================= Test Config ==================================
i:                  1 (default)
f:                  2.1 (default)
d:                  3.2 (default)
b:                  true (default)
u8:                 4 (default)
s:                  test string (default)
vec:                [1, 2, 3] (default)
map:                {a: 1, b: 2, c: 3} (default)
set:                [1.1, 2.2, 3.3] (default)
mat:                [[1, 0, 0],
                     [0, 1, 0],
                     [0, 0, 1]] (default)
my_enum:            A (default)
my_strange_enum:    X (default)
A ridiculously long field name that will not be wrapped: Short Value (default)
A ridiculously long field name that will also not be wrapped:
                    A really really really ridiculously long string that will be
                    wrapped. (default)
A really really really really really really ridiculously long field name that wi
ll be wrapped:      A really really really ridiculously long string that will al
                    so be wrapped. (default)
sub_config [SubConfig] (default):
   i:               1 (default)
   sub_sub_config [SubSubConfig] (default):
      i:            1 (default)
sub_sub_config [SubSubConfig] (default):
   i:               1 (default)
================================================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);

  TestConfig modified_config;
  internal::Visitor::setValues(modified_config, DefaultConfig::modifiedValues());
  const internal::MetaData modified_data = internal::Visitor::getValues(modified_config);
  formatted = internal::Formatter::formatConfig(modified_data);
  expected = R"""(================================= Test Config ==================================
i:                  2
f:                  -1
d:                  3.14159
b:                  false
u8:                 255
s:                  a different test string
vec:                [2, 3, 4, 5]
map:                {x: 24, y: 25, z: 26}
set:                [11.11, 22.22, 33.33, 44.44]
mat:                [[1, 2, 3],
                     [4, 5, 6],
                     [7, 8, 9]]
my_enum:            B
my_strange_enum:    Z
A ridiculously long field name that will not be wrapped: Short Value (default)
A ridiculously long field name that will also not be wrapped:
                    A really really really ridiculously long string that will be
                    wrapped. (default)
A really really really really really really ridiculously long field name that wi
ll be wrapped:      A really really really ridiculously long string that will al
                    so be wrapped. (default)
sub_config [SubConfig]:
   i:               2
   sub_sub_config [SubSubConfig]:
      i:            3
sub_sub_config [SubSubConfig]:
   i:               4
================================================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, FormatArrayChecks) {
  ConfigUsingArrays config;
  config.arr.emplace_back().f = -1.f;
  config.arr.emplace_back().f = -2.f;
  config.arr.emplace_back().f = -3.f;

  const internal::MetaData data = internal::Visitor::getChecks(config);
  const std::string formatted = internal::Formatter::formatErrors(data);
  const std::string expected = R"""( 'ConfigUsingArrays':
============================== ConfigUsingArrays ===============================
Warning: Check [1/3] failed for 'arr[0].f': param >= 0 (is: '-1').
Warning: Check [2/3] failed for 'arr[1].f': param >= 0 (is: '-2').
Warning: Check [3/3] failed for 'arr[2].f': param >= 0 (is: '-3').
================================================================================)""";
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, FormatArrayConfigs) {
  ConfigUsingArrays config;
  config.arr.emplace_back().f = 1.f;
  config.arr.emplace_back().f = 2.f;
  config.arr.emplace_back().f = 3.f;

  const internal::MetaData data = internal::Visitor::getValues(config);
  const std::string formatted = internal::Formatter::formatConfig(data);
  const std::string expected = R"""(============================== ConfigUsingArrays ===============================
arr[0] [ArrayConfig]:
   f:               1
arr[1] [ArrayConfig]:
   f:               2
arr[2] [ArrayConfig]:
   f:               3
================================================================================)""";
}

}  // namespace config::test
