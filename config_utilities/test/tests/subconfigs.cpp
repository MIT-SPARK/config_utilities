#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"


namespace config::test {

struct FakeVector {
  int x = 0;
  int y = 0;
  int z = 0;
};

void declare_config(FakeVector& vector) {
  name("FakeVector");
  field(vector.x, "x");
  field(vector.y, "y");
  field(vector.z, "z");
}

struct VectorConfig {
  FakeVector a;
  FakeVector b;
};

struct BadVectorConfig : public VectorConfig {};

void declare_config(VectorConfig& config) {
  name("VectorConfig");
  field(config.a, "a");
  field(config.b, "b");
}

void declare_config(BadVectorConfig& config) {
  name("VectorConfig");
  field(config.a, "a");
  field(config.b, "b", false);
}

TEST(Subconfigs, NamespacedSubconfig) {
  const std::string yaml_str = R"(
x: 1
y: 2
z: 3
a: {x: 4, y: 5, z: 6}
b: {x: -1, y: -2, z: -3})";
  const auto node = YAML::Load(yaml_str);

  const auto good_config = fromYaml<VectorConfig>(node);
  const auto bad_config = fromYaml<BadVectorConfig>(node);

  EXPECT_EQ(good_config.a.x, 4);
  EXPECT_EQ(good_config.a.y, 5);
  EXPECT_EQ(good_config.a.z, 6);
  EXPECT_EQ(bad_config.a.x, 4);
  EXPECT_EQ(bad_config.a.y, 5);
  EXPECT_EQ(bad_config.a.z, 6);

  EXPECT_EQ(good_config.b.x, -1);
  EXPECT_EQ(good_config.b.y, -2);
  EXPECT_EQ(good_config.b.z, -3);
  EXPECT_EQ(bad_config.b.x, 1);
  EXPECT_EQ(bad_config.b.y, 2);
  EXPECT_EQ(bad_config.b.z, 3);
}

}  // namespace config::test
