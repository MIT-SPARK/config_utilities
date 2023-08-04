#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"
#include "config_utilities/printing.h"

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

struct FlatVectorConfig : public VectorConfig {};
struct NamespacedVectorConfig : public VectorConfig {};

void declare_config(VectorConfig& config) {
  name("VectorConfig");
  field(config.a, "a");
  field(config.b, "b");
}

void declare_config(FlatVectorConfig& config) {
  name("FlatVectorConfig");
  field(config.a, "a", false);
  field(config.b, "b", false);
}

void declare_config(NamespacedVectorConfig& config) {
  name("NamespacedVectorConfig");
  field(config.a, "a");
  field(config.b, "b", true, "a");
}

bool operator==(const FakeVector& lhs, const FakeVector& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator==(const VectorConfig& lhs, const VectorConfig& rhs) { return lhs.a == rhs.a && lhs.b == rhs.b; }

VectorConfig makeExpected(int a_x, int a_y, int a_z, int b_x, int b_y, int b_z) {
  FakeVector a{a_x, a_y, a_z};
  FakeVector b{b_x, b_y, b_z};
  return {a, b};
}

void PrintTo(const VectorConfig& conf, std::ostream* os) { *os << toString(conf); }
void PrintTo(const FlatVectorConfig& conf, std::ostream* os) { *os << toString(conf); }
void PrintTo(const NamespacedVectorConfig& conf, std::ostream* os) { *os << toString(conf); }

TEST(Subconfigs, SubconfigNamespacing) {
  const std::string yaml_str = R"(
x: 1
y: 2
z: 3
a: {x: 4, y: 5, z: 6}
b: {x: -1, y: -2, z: -3})";
  const auto node = YAML::Load(yaml_str);

  auto nested_config = fromYaml<VectorConfig>(node);
  auto flat_config = fromYaml<FlatVectorConfig>(node);
  auto namespaced_config = fromYaml<NamespacedVectorConfig>(node);
  EXPECT_EQ(makeExpected(4, 5, 6, -1, -2, -3), nested_config);
  EXPECT_EQ(makeExpected(1, 2, 3, 1, 2, 3), flat_config);
  EXPECT_EQ(makeExpected(4, 5, 6, 4, 5, 6), namespaced_config);
}

}  // namespace config::test
