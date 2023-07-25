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

TEST(AslFormatter, dataToString) {
  YAML::Node data = internal::Visitor::getValues(TestConfig()).data;
  EXPECT_EQ(internal::dataToString(data["i"]), "1");
  EXPECT_EQ(internal::dataToString(data["f"]), "2.1");
  EXPECT_EQ(internal::dataToString(data["d"]), "3.2");
  EXPECT_EQ(internal::dataToString(data["b"]), "true");
  EXPECT_EQ(internal::dataToString(data["u8"]), "4");
  EXPECT_EQ(internal::dataToString(data["s"]), "test string");
  EXPECT_EQ(internal::dataToString(data["vec"]), "[1, 2, 3]");
  EXPECT_EQ(internal::dataToString(data["map"]), "{a: 1, b: 2, c: 3}");
  EXPECT_EQ(internal::dataToString(data["set"]), "[1.1, 2.2, 3.3]");
  EXPECT_EQ(internal::dataToString(data["mat"]), "[[1, 0, 0], [0, 1, 0], [0, 0, 1]]");
  YAML::Node nested_set;
  nested_set["a"]["x"] = 1;
  nested_set["a"]["y"] = 2;
  nested_set["b"]["x"] = 3;
  nested_set["b"]["y"] = 4;
  EXPECT_EQ(internal::dataToString(nested_set), "{a: {x: 1, y: 2}, b: {x: 3, y: 4}}");
}

TEST(AslFormatter, formatErrors) {
  internal::MetaData data;
  data.name = "Config 1";
  data.errors.push_back("Error 1");
  data.errors.push_back("Error 2");
  internal::MetaData& d = data.sub_configs.emplace_back();
  d.name = "Config 2";
  d.errors.push_back("Error 3");
  d.errors.push_back("Error 4");
  internal::MetaData& d2 = data.sub_configs.emplace_back();
  internal::MetaData& d3 = d2.sub_configs.emplace_back();
  d3.name = "Config 3";
  d3.errors.push_back("Error 5");
  internal::MetaData& d4 = d3.sub_configs.emplace_back();
  d4.name = "Config 4";
  d4.errors.push_back("Error 6");

  std::string formatted = internal::Formatter::formatErrors(data);
  EXPECT_EQ(countLines(formatted), 9);

  std::string expected = R"""( 'Config 1':
=================================== Config 1 ===================================
Warning: Error 1
Warning: Error 2
Warning: Error 3
Warning: Error 4
Warning: Error 5
Warning: Error 6
================================================================================)""";
  EXPECT_EQ(formatted, expected);

  Settings().inline_subconfig_field_names = false;
  formatted = internal::Formatter::formatErrors(data);
  EXPECT_EQ(countLines(formatted), 12);

  expected = R"""( 'Config 1':
=================================== Config 1 ===================================
Warning: Error 1
Warning: Error 2
----------------------------------- Config 2 -----------------------------------
Warning: Error 3
Warning: Error 4
----------------------------------- Config 3 -----------------------------------
Warning: Error 5
----------------------------------- Config 4 -----------------------------------
Warning: Error 6
================================================================================)""";
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, formatChecks) {
  DefaultConfig config;
  config.i = -1;
  config.f = -1.f;
  config.d = 100.0;
  // TODO(lschmid): u8 formatting is currently not supported in the checks. Maybe using the field declaration and yaml
  // parser could potentially resolve this.
  // config.u8 = 26;
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
Warning: Check [7/8] failed: Param 'vec' must b of size '3'.
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
Warning: Check [7/11] failed: Param 'vec' must b of size '3'.
Warning: Check [8/11] failed for 'd': param within [0, 500] (is: '1000').
Warning: Check [9/11] failed for 'sub_config.i': param > 0 (is: '-1').
Warning: Check [10/11] failed for 'sub_config.sub_sub_config.i': param > 0 (is:
         '-1').
Warning: Check [11/11] failed for 'sub_sub_config.i': param > 0 (is: '-1').
================================================================================)""";
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, formatConfig) {
  internal::MetaData data = internal::Visitor::getValues(TestConfig());

  Settings().indicate_default_values = false;
  Settings().indicate_units = false;
  Settings().inline_subconfig_field_names = true;
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
sub_config [SubConfig]:
   i:                         1
   sub_sub_config [SubSubConfig]:
      i:                      1
sub_sub_config [SubSubConfig]:
   i:                         1
A ridiculously long field name that will not be wrapped: Short Value
A ridiculously long field name that will also not be wrapped:
                              A really really really ridiculously long string th
                              at will be wrapped.
A really really really really really really ridiculously long field name that wi
ll be wrapped:                A really really really ridiculously long string th
                              at will also be wrapped.
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
sub_config [SubConfig]:
   i:                         1
   sub_sub_config [SubSubConfig]:
      i:                      1
sub_sub_config [SubSubConfig]:
   i:                         1
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
sub_config [SubConfig]:
   i:               1
   sub_sub_config [SubSubConfig]:
      i:            1
sub_sub_config [SubSubConfig]:
   i:               1
A ridiculously long field name that will not be wrapped: Short Value
A ridiculously long field name that will also not be wrapped:
                    A really really really ridiculously long string that will be
                    wrapped.
A really really really really really really ridiculously long field name that wi
ll be wrapped:      A really really really ridiculously long string that will al
                    so be wrapped.
================================================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, formatUnits) {
  Settings().indicate_default_values = false;
  Settings().indicate_units = true;
  Settings().inline_subconfig_field_names = true;
  Settings().print_width = 80;  // force print width to be consistent for tests

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
sub_config [SubConfig]:
   i:               1
   sub_sub_config [SubSubConfig]:
      i:            1
sub_sub_config [SubSubConfig]:
   i:               1
A ridiculously long field name that will not be wrapped [ms]: Short Value
A ridiculously long field name that will also not be wrapped [custom unit]:
                    A really really really ridiculously long string that will be
                    wrapped.
A really really really really really really ridiculously long field name that wi
ll be wrapped [and has a long unit]:
                    A really really really ridiculously long string that will al
                    so be wrapped.
================================================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, formatDefaultValues) {
  Settings().indicate_default_values = true;
  Settings().indicate_units = false;
  Settings().inline_subconfig_field_names = true;

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
sub_config [SubConfig] (default):
   i:               1 (default)
   sub_sub_config [SubSubConfig] (default):
      i:            1 (default)
sub_sub_config [SubSubConfig] (default):
   i:               1 (default)
A ridiculously long field name that will not be wrapped: Short Value (default)
A ridiculously long field name that will also not be wrapped:
                    A really really really ridiculously long string that will be
                    wrapped. (default)
A really really really really really really ridiculously long field name that wi
ll be wrapped:      A really really really ridiculously long string that will al
                    so be wrapped. (default)
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
d:                  3.1415926
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
sub_config [SubConfig]:
   i:               2
   sub_sub_config [SubSubConfig]:
      i:            3
sub_sub_config [SubSubConfig]:
   i:               4
A ridiculously long field name that will not be wrapped: Short Value (default)
A ridiculously long field name that will also not be wrapped:
                    A really really really ridiculously long string that will be
                    wrapped. (default)
A really really really really really really ridiculously long field name that wi
ll be wrapped:      A really really really ridiculously long string that will al
                    so be wrapped. (default)
================================================================================)""";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);
}

}  // namespace config::test
