#include <sstream>

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

  expectEqual(data, data);

  YAML::Node data_1 = YAML::Clone(data);
  data_1["a"]["b"]["c"] = 2;
  EXPECT_FALSE(internal::isEqual(data, data_1));

  YAML::Node data_2 = internal::lookupNamespace(data, "");
  // NOTE(lschmid): lookupNamespace returns a pointer, so this should be identity.
  EXPECT_TRUE(data == data_2);
  expectEqual(data, data_2);

  YAML::Node b = internal::lookupNamespace(data, "a/b");
  // NOTE(lschmid): lookupNamespace returns a pointer, so this should be identity.
  EXPECT_TRUE(b == data["a"]["b"]);
  expectEqual(b, data["a"]["b"]);

  YAML::Node b2 = internal::lookupNamespace(YAML::Clone(data), "a/b");

  expectEqual(b2, data["a"]["b"]);

  YAML::Node c = internal::lookupNamespace(data, "a/b/c");
  EXPECT_TRUE(c.IsScalar());
  EXPECT_EQ(c.as<int>(), 1);

  YAML::Node invalid = internal::lookupNamespace(data, "a/b/c/d");
  EXPECT_FALSE(invalid.IsDefined());
  EXPECT_FALSE(static_cast<bool>(invalid));

  // Make sure the input node is not modified.
  expectEqual(data, createData());
}

TEST(YamlParsing, moveDownNamespace) {
  YAML::Node data = createData();

  internal::moveDownNamespace(data, "");
  expectEqual(data, createData());

  YAML::Node expected_data;
  expected_data["a"]["b"]["c"] = createData();
  internal::moveDownNamespace(data, "a/b/c");
  expectEqual(data, expected_data);
}

TEST(YamlParsing, parsefromYaml) {
  DefaultConfig config;
  internal::YamlParser parser;
  YAML::Node data = loadResource("modified_config_values");
  parser.setNode(data);

  parser.fromYaml("i", config.i, "", "");
  EXPECT_EQ(config.i, 2);

  parser.fromYaml("f", config.f, "", "");
  EXPECT_EQ(config.f, -1.f);

  parser.fromYaml("d", config.d, "", "");
  EXPECT_EQ(config.d, 3.1415926);

  parser.fromYaml("b", config.b, "", "");
  EXPECT_EQ(config.b, false);

  parser.fromYaml("u8", config.u8, "", "");
  EXPECT_EQ(config.u8, 255);

  parser.fromYaml("s", config.s, "", "");
  EXPECT_EQ(config.s, "a different test string");

  parser.fromYaml("vec", config.vec, "", "");
  EXPECT_EQ(config.vec, std::vector<int>({2, 3, 4, 5}));

  parser.fromYaml("map", config.map, "", "");
  const std::map<std::string, int> map({{"x", 24}, {"y", 25}, {"z", 26}});
  EXPECT_EQ(config.map, map);

  parser.fromYaml("set", config.set, "", "");
  EXPECT_EQ(config.set, std::set<float>({11.11, 22.22, 33.33, 44.44}));

  parser.fromYaml("mat", config.mat, "", "");
  Eigen::Matrix3d mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  EXPECT_EQ(config.mat, mat);

  // TODO(lschmid): Could add tests that correct warnings are issued and that unset params don't modfy the config but
  // not important for now.
}

TEST(YamlParsing, setValues) {
  YAML::Node data = loadResource("modified_config_values");
  DefaultConfig config;
  internal::MetaData meta_data = internal::Visitor::setValues(config, data);
  expectModifiedValues(config);
  EXPECT_FALSE(meta_data.hasErrors());

  // Make sure the input node is not modified.
  expectEqual(data, loadResource("modified_config_values"));

  // Test that the config is not modified if no data is found is empty.
  data = YAML::Node();
  meta_data = internal::Visitor::setValues(config, data);
  expectModifiedValues(config);
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
  expectModifiedValues(config);
  EXPECT_TRUE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 8ul);
}

TEST(YamlParsing, getValues) {
  DefaultConfig config;
  internal::MetaData meta_data = internal::Visitor::getValues(config);

  expextDefaultValues(config);
  expectEqual(meta_data.data, loadResource("default_config_values"));
  EXPECT_FALSE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 0ul);
  meta_data.performOnAll([](const internal::MetaData& d) {
    for (const auto& field : d.field_infos) {
      EXPECT_TRUE(field.is_default);
    }
  });
  EXPECT_EQ(meta_data.name, "DefaultConfig");

  internal::Visitor::setValues(config, loadResource("modified_config_values"));
  meta_data = internal::Visitor::getValues(config);
  expectModifiedValues(config);
  expectEqual(meta_data.data, loadResource("modified_config_values"));
  EXPECT_FALSE(meta_data.hasErrors());
  EXPECT_EQ(meta_data.errors.size(), 0ul);
  meta_data.performOnAll([](const internal::MetaData& d) {
    for (const auto& field : d.field_infos) {
      EXPECT_FALSE(field.is_default);
    }
  });
}
TEST(YamlParsing, configFromYaml) {
  const YAML::Node data = loadResource("modified_config_values");
  auto config = fromYaml<DefaultConfig>(data);
  expectModifiedValues(config);

  // Make sure the input node is not modified.
  expectEqual(data, loadResource("modified_config_values"));
}

TEST(YamlParsing, configToYAML) {
  DefaultConfig config;
  expectEqual(toYaml(config), loadResource("default_config_values"));

  config = fromYaml<DefaultConfig>(loadResource("modified_config_values"));
  expectEqual(toYaml(config), loadResource("modified_config_values"));
}

}  // namespace config::test
