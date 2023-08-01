#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/parsing/yaml.h"

namespace config::test {

struct ArrConfig {
  std::string s;
  float f;
};

void declare_config(ArrConfig& config) {
  name("ArrConfig");
  field(config.s, "s");
  field(config.f, "f");
}

TEST(ConfigArrays, FromYamlSeq) {
  const std::string yaml_seq = R"(
- s: "a"
  f: 1.0
- s: "b"
  f: 2.0
- s: "c"
  f: 3.0
)";
  YAML::Node node = YAML::Load(yaml_seq);

  auto configs = fromYaml<std::vector<ArrConfig>>(node);
  EXPECT_EQ(configs.size(), 3);
  EXPECT_EQ(configs[0].s, "a");
  EXPECT_EQ(configs[0].f, 1.0f);
  EXPECT_EQ(configs[1].s, "b");
  EXPECT_EQ(configs[1].f, 2.0f);
  EXPECT_EQ(configs[2].s, "c");
  EXPECT_EQ(configs[2].f, 3.0f);
}

TEST(ConfigArrays, FromYamlMap) {
  const std::string yaml_map = R"(
x:
  s: "a"
  f: 1.0
y:
  s: "b"
  f: 2.0
z:
  s: "c"
  f: 3.0
)";
  YAML::Node node = YAML::Load(yaml_map);

  auto configs = fromYaml<std::vector<ArrConfig>>(node);
  EXPECT_EQ(configs.size(), 3);
  EXPECT_EQ(configs[0].s, "a");
  EXPECT_EQ(configs[0].f, 1.0f);
  EXPECT_EQ(configs[1].s, "b");
  EXPECT_EQ(configs[1].f, 2.0f);
  EXPECT_EQ(configs[2].s, "c");
  EXPECT_EQ(configs[2].f, 3.0f);
}

}  // namespace config::test
