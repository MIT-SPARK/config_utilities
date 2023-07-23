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
  d2.name = "Config 3";
  d2.errors.push_back("Error 5");
  internal::MetaData& d3 = d2.sub_configs.emplace_back();
  d3.name = "Config 4";
  d3.errors.push_back("Error 6");

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

  Settings().index_subconfig_field_names = false;
  formatted = internal::Formatter::formatErrors(data);
  EXPECT_EQ(countLines(formatted), 12);

  expected = R"""( 'Config 1':
=================================== Config 1 ===================================
Warning: Error 1
Warning: Error 2
=================================== Config 2 ===================================
Warning: Error 3
Warning: Error 4
=================================== Config 3 ===================================
Warning: Error 5
=================================== Config 4 ===================================
Warning: Error 6
================================================================================)""";
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, formatConfig) {
  internal::MetaData data = internal::Visitor::getValues(TestConfig());

  Settings().indicate_default_values = false;
  Settings().indicate_units = false;
  Settings().index_subconfig_field_names = true;
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
                               string that will al
                              so be wrapped.
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
  Settings().index_subconfig_field_names = true;

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
  Settings().index_subconfig_field_names = true;

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
