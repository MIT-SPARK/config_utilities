#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/formatting/asl.h"
#include "config_utilities/internal/meta_data.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/test/default_configs.h"
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

  std::string expected =
      " 'Config 1':\n=================================== Config 1 ===================================\nWarning: "
      "Error 1\nWarning: Error 2\nWarning: Error 3\nWarning: Error 4\nWarning: Error 5\nWarning: Error "
      "6\n================================================================================";
  EXPECT_EQ(formatted, expected);

  Settings().index_subconfig_field_names = false;
  formatted = internal::Formatter::formatErrors(data);
  EXPECT_EQ(countLines(formatted), 12);

  expected =
      " 'Config 1':\n=================================== Config 1 ===================================\nWarning: "
      "Error 1\nWarning: Error 2\n=================================== Config 2 "
      "===================================\nWarning: Error 3\nWarning: Error 4\n=================================== "
      "Config 3 ===================================\nWarning: Error 5\n=================================== Config 4 "
      "===================================\nWarning: Error "
      "6\n================================================================================";
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, formatConfig) {
  internal::MetaData data = internal::Visitor::getValues(TestConfig());

  Settings().indicate_default_values = false;
  Settings().indicate_units = false;
  Settings().index_subconfig_field_names = true;
  std::string formatted = internal::Formatter::formatConfig(data);
  std::string expected =
      "================================= Test Config ==================================\ni:                            "
      "1\nf:                            2.1\nd:                            3.2\nb:                            "
      "true\nu8:                           4\ns:                            test string\nvec:                          "
      "[1, 2, 3]\nmap:                          {a: 1, b: 2, c: 3}\nset:                          [1.1, 2.2, "
      "3.3]\nmat:                          [[1, 0, 0],\n                               [0, 1, 0],\n                    "
      "           [0, 0, 1]]\nmy_enum:                      A\nmy_strange_enum:              X\nsub_config "
      "[SubConfig]:\n   i:                         1\n   sub_sub_config [SubSubConfig]:\n      i:                      "
      "1\nsub_sub_config [SubSubConfig]:\n   i:                         1\nA ridiculously long field name that will "
      "not be wrapped: Short Value\nA ridiculously long field name that will also not be wrapped: \n                   "
      " "
      "          A really really really ridiculously long string th\n                              at will be "
      "wrapped.\nA really really really really really really ridiculously long field name that wi\nll be wrapped:      "
      "          A really really really ridiculously long string th\n                              at will also be "
      "wrapped.\n================================================================================";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);

  Settings().print_width = 50;
  formatted = internal::Formatter::formatConfig(data);
  expected =
      "================== Test Config ===================\ni:                            1\nf:                         "
      "   2.1\nd:                            3.2\nb:                            true\nu8:                           "
      "4\ns:                            test string\nvec:                          [1, 2, 3]\nmap:                     "
      "     {a: 1, b: 2, c: 3}\nset:                          [1.1, 2.2, 3.3]\nmat:                          [[1, 0, "
      "0],\n                               [0, 1, 0],\n                               [0, 0, 1]]\nmy_enum:             "
      "         A\nmy_strange_enum:              X\nsub_config [SubConfig]:\n   i:                         1\n   "
      "sub_sub_config [SubSubConfig]:\n      i:                      1\nsub_sub_config [SubSubConfig]:\n   i:          "
      "               1\nA ridiculously long field name that will not be wr\napped:                        Short "
      "Value\nA ridiculously long field name that will also not \nbe wrapped:                   A really really real\n "
      " "
      "                            ly ridiculously long\n                               string that will be\n          "
      "                     wrapped.\nA really really really really really really ridicu\nlously long field name that "
      "will be wrapped: \n                              A really really real\n                              ly "
      "ridiculously long\n                               string that will al\n                              so be "
      "wrapped.\n==================================================";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);

  Settings().print_width = 80;
  Settings().print_indent = 20;
  formatted = internal::Formatter::formatConfig(data);
  expected =
      "================================= Test Config ==================================\ni:                  1\nf:     "
      "             2.1\nd:                  3.2\nb:                  true\nu8:                 4\ns:                  "
      "test string\nvec:                [1, 2, 3]\nmap:                {a: 1, b: 2, c: 3}\nset:                [1.1, "
      "2.2, 3.3]\nmat:                [[1, 0, 0],\n                     [0, 1, 0],\n                     [0, 0, "
      "1]]\nmy_enum:            A\nmy_strange_enum:    X\nsub_config [SubConfig]:\n   i:               1\n   "
      "sub_sub_config [SubSubConfig]:\n      i:            1\nsub_sub_config [SubSubConfig]:\n   i:               1\nA "
      "ridiculously long field name that will not be wrapped: Short Value\nA ridiculously long field name that will "
      "also not be wrapped: \n                    A really really really ridiculously long string that will be\n       "
      " "
      "             wrapped.\nA really really really really really really ridiculously long field name that wi\nll be "
      "wrapped:      A really really really ridiculously long string that will al\n                    so be "
      "wrapped.\n================================================================================";
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
      "================================= Test Config ==================================\ni [m]:              1\nf [s]: "
      "             2.1\nd [m/s]:            3.2\nb:                  true\nu8:                 4\ns:                  "
      "test string\nvec [frames]:       [1, 2, 3]\nmap:                {a: 1, b: 2, c: 3}\nset:                [1.1, "
      "2.2, 3.3]\nmat:                [[1, 0, 0],\n                     [0, 1, 0],\n                     [0, 0, "
      "1]]\nmy_enum:            A\nmy_strange_enum:    X\nsub_config [SubConfig]:\n   i:               1\n   "
      "sub_sub_config [SubSubConfig]:\n      i:            1\nsub_sub_config [SubSubConfig]:\n   i:               1\nA "
      "ridiculously long field name that will not be wrapped [ms]: Short Value\nA ridiculously long field name that "
      "will also not be wrapped [custom unit]: \n                    A really really really ridiculously long string "
      "that will be\n                     wrapped.\nA really really really really really really ridiculously long "
      "field name that wi\nll be wrapped [and has a long unit]: \n                    A really really really "
      "ridiculously long string that will al\n                    so be "
      "wrapped.\n================================================================================";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);
}

TEST(AslFormatter, formatDefaultValues) {
  Settings().indicate_default_values = true;
  Settings().indicate_units = false;
  Settings().index_subconfig_field_names = true;

  const internal::MetaData default_data = internal::Visitor::getValues(TestConfig());
  std::string formatted = internal::Formatter::formatConfig(default_data);
  std::string expected =
      "================================= Test Config ==================================\ni:                  1 "
      "(default)\nf:                  2.1 (default)\nd:                  3.2 (default)\nb:                  true "
      "(default)\nu8:                 4 (default)\ns:                  test string (default)\nvec:                [1, "
      "2, 3] (default)\nmap:                {a: 1, b: 2, c: 3} (default)\nset:                [1.1, 2.2, 3.3] "
      "(default)\nmat:                [[1, 0, 0],\n                     [0, 1, 0],\n                     [0, 0, 1]] "
      "(default)\nmy_enum:            A (default)\nmy_strange_enum:    X (default)\nsub_config [SubConfig] "
      "(default):\n   i:               1 (default)\n   sub_sub_config [SubSubConfig] (default):\n      i:            1 "
      "(default)\nsub_sub_config [SubSubConfig] (default):\n   i:               1 (default)\nA ridiculously long field "
      "name that will not be wrapped: Short Value (default)\nA ridiculously long field name that will also not be "
      "wrapped: \n                    A really really really ridiculously long string that will be\n                   "
      " "
      " wrapped. (default)\nA really really really really really really ridiculously long field name that wi\nll be "
      "wrapped:      A really really really ridiculously long string that will al\n                    so be wrapped. "
      "(default)\n================================================================================";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);

  TestConfig modified_config;
  internal::Visitor::setValues(modified_config, loadResource("modified_config_values"));
  const internal::MetaData modified_data = internal::Visitor::getValues(modified_config);
  formatted = internal::Formatter::formatConfig(modified_data);
  expected =
      "================================= Test Config ==================================\ni:                  2\nf:     "
      "             -1\nd:                  3.1415926\nb:                  false\nu8:                 255\ns:          "
      "        a different test string\nvec:                [2, 3, 4, 5]\nmap:                {x: 24, y: 25, z: "
      "26}\nset:                [11.11, 22.22, 33.33, 44.44]\nmat:                [[1, 2, 3],\n                     "
      "[4, 5, 6],\n                     [7, 8, 9]]\nmy_enum:            B\nmy_strange_enum:    Z\nsub_config "
      "[SubConfig]:\n   i:               2\n   sub_sub_config [SubSubConfig]:\n      i:            3\nsub_sub_config "
      "[SubSubConfig]:\n   i:               4\nA ridiculously long field name that will not be wrapped: Short Value "
      "(default)\nA ridiculously long field name that will also not be wrapped: \n                    A really really "
      "really ridiculously long string that will be\n                     wrapped. (default)\nA really really really "
      "really really really ridiculously long field name that wi\nll be wrapped:      A really really really "
      "ridiculously long string that will al\n                    so be wrapped. "
      "(default)\n================================================================================";
  EXPECT_EQ(formatted.size(), expected.size());
  EXPECT_EQ(formatted, expected);
}

}  // namespace config::test
