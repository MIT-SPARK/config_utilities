#include "config_utilities/types/conversions.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"

namespace config::test {

struct ConversionStruct {
  int num_threads = 1;
  int some_other_int = 1;
  char some_character = 'a';
  uint8_t some_number = 'a';
};

void declare_config(ConversionStruct& conf) {
  // note that this is probably not a very useful way to declare a config
  field<ThreadNumConversion>(conf.num_threads, "num_threads");
  field(conf.some_other_int, "num_threads");
  field<CharConversion>(conf.some_character, "some_character");
  field(conf.some_number, "some_character");
}

// tests that we pull the right character from a string
TEST(Conversions, CharConversionCorrect) {
  std::string normal = "h";
  std::string too_long = "hello";
  std::string empty = "";

  // TODO(nathan) test for logging
  EXPECT_EQ(CharConversion::fromIntermediate(normal), 'h');
  EXPECT_EQ(CharConversion::fromIntermediate(too_long), 'h');
  EXPECT_EQ(CharConversion::fromIntermediate(empty), '\0');
}

// tests that we can default autodetection of the number of cores
TEST(Conversions, ThreadNumConversionCorrect) {
  int auto_detect = -1;
  int manually_specified = 2;
  EXPECT_GT(ThreadNumConversion::fromIntermediate(auto_detect), 0);
  EXPECT_EQ(ThreadNumConversion::fromIntermediate(manually_specified), manually_specified);
}

// tests that conversions work as expected when parsing a config
TEST(Conversions, ConvertFields) {
  const std::string yaml_string = R"yaml(
num_threads: -1
some_character: c
)yaml";
  const auto node = YAML::Load(yaml_string);
  const auto conf = fromYaml<ConversionStruct>(node);

  EXPECT_GT(conf.num_threads, 0);
  EXPECT_EQ(conf.some_other_int, -1);
  EXPECT_EQ(conf.some_character, 'c');
  EXPECT_EQ(conf.some_number, static_cast<uint16_t>('a'));
}

// tests that conversions don't interfere when parsing a config where they don't apply
TEST(Conversions, ConvertFieldsPassthrough) {
  const std::string yaml_string = R"yaml(
num_threads: 5
some_character: 5
)yaml";
  const auto node = YAML::Load(yaml_string);
  const auto conf = fromYaml<ConversionStruct>(node);

  EXPECT_EQ(conf.num_threads, 5);
  EXPECT_EQ(conf.some_other_int, 5);
  EXPECT_EQ(conf.some_character, '5');
  EXPECT_EQ(conf.some_number, 5);
}

}  // namespace config::test
