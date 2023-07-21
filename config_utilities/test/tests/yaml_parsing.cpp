#include <gtest/gtest.h>

#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/default_configs.h"
#include "config_utilities/test/utils.h"

namespace config::test {

YAML::Node createData() {
  YAML::Node data;
  data["a"]["b"]["c"] = 1;
  data["a"]["b"]["d"] = "test";
  data["a"]["b"]["e"] = std::vector<float>({1, 2, 3});
  data["a"]["b"]["f"] = std::map<std::string, int>({{"1_str", 1}, {"2_str", 2}});
  data["a"]["g"] = 3;
  return data;
}

TEST(YamlParsing, lookupNamespace) {
  YAML::Node data = createData();

  EXPECT_TRUE(internal::isEqual(data, data));

  YAML::Node data_1 = YAML::Clone(data);
  data_1["a"]["b"]["c"] = 2;
  EXPECT_FALSE(internal::isEqual(data, data_1));

  YAML::Node data_2 = internal::lookupNamespace(data, "");
  // NOTE(lschmid): lookupNamespace returns a pointer, so this should be identity.
  EXPECT_TRUE(data == data_2);
  EXPECT_TRUE(internal::isEqual(data, data_2));

  YAML::Node b = internal::lookupNamespace(data, "a/b");
  // NOTE(lschmid): lookupNamespace returns a pointer, so this should be identity.
  EXPECT_TRUE(b == data["a"]["b"]);
  EXPECT_TRUE(internal::isEqual(b, data["a"]["b"]));

  YAML::Node b2 = internal::lookupNamespace(YAML::Clone(data), "a/b");

  EXPECT_TRUE(internal::isEqual(b2, data["a"]["b"]));

  YAML::Node c = internal::lookupNamespace(data, "a/b/c");
  EXPECT_TRUE(c.IsScalar());
  EXPECT_EQ(c.as<int>(), 1);

  YAML::Node invalid = internal::lookupNamespace(data, "a/b/c/d");
  EXPECT_FALSE(invalid.IsDefined());
  EXPECT_FALSE(static_cast<bool>(invalid));

  // Make sure the input node is not modified.
  EXPECT_TRUE(internal::isEqual(data, createData()));
}

TEST(YamlParsing, moveDownNamespace) {
  YAML::Node data = createData();

  internal::moveDownNamespace(data, "");
  EXPECT_TRUE(internal::isEqual(data, createData()));

  YAML::Node expected_data;
  expected_data["a"]["b"]["c"] = createData();
  internal::moveDownNamespace(data, "a/b/c");
  EXPECT_TRUE(internal::isEqual(data, expected_data));
}

TEST(YamlParsing, configFromYaml) {
  const YAML::Node data = loadResource("modified_config_values");
  auto config = fromYaml<DefaultConfig>(data);
  EXPECT_EQ(config.i, 2);
  EXPECT_EQ(config.f, -1.f);
  EXPECT_EQ(config.d, 3.1415926);
  EXPECT_EQ(config.b, false);
  EXPECT_EQ(config.u8, 255);
  EXPECT_EQ(config.s, "a different test string");
  EXPECT_EQ(config.vec, std::vector<int>({2, 3, 4, 5}));
  const std::map<std::string, int> map({{"x", 24}, {"y", 25}, {"z", 26}});
  EXPECT_EQ(config.map, map);
  EXPECT_EQ(config.set, std::set<float>({11.11, 22.22, 33.33, 44.44}));
  Eigen::Matrix3d mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  EXPECT_EQ(config.mat, mat);
  EXPECT_EQ(config.my_enum, DefaultConfig::Enum::kB);
  EXPECT_EQ(config.my_strange_enum, DefaultConfig::StrangeEnum::kZ);

  // Make sure the input node is not modified.
  EXPECT_TRUE(internal::isEqual(data, loadResource("modified_config_values")));
}

TEST(YamlParsing, configToYaml) {
  DefaultConfig config;
  EXPECT_TRUE(internal::isEqual(toYaml(config), loadResource("default_config_values")));

  auto modified_config = fromYaml<DefaultConfig>(loadResource("modified_config_values"));
  EXPECT_TRUE(internal::isEqual(toYaml(modified_config), loadResource("modified_config_values")));
}

}  // namespace config::test
