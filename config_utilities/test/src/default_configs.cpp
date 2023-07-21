
#include "config_utilities/test/default_configs.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"

namespace config::test {

void declare_config(SubSubConfig& config) {
  config::name("SubSubConfig");
  config::field(config.i, "i");
  config::checkGT(config.i, 0, "i");
}

void declare_config(SubConfig& config) {
  config::name("SubConfig");
  config::field(config.i, "i");
  config::subconfig(config.sub_sub_config, "sub_sub_config", "nested_sub_ns");
  config::checkGT(config.i, 0, "i");
}

void declare_config(DefaultConfig& config) {
  config::name("DefaultConfig");
  config::field(config.i, "i");
  config::field(config.f, "f");
  config::field(config.d, "d");
  config::field(config.b, "b");
  config::field(config.u8, "u8");
  config::field(config.s, "s");
  config::field(config.vec, "vec");
  config::field(config.map, "map");
  config::field(config.set, "set");
  config::field(config.mat, "mat");
  config::enum_field(config.my_enum, "my_enum", {"A", "B", "C"});
  config::enum_field(config.my_strange_enum,
                     "my_strange_enum",
                     {{DefaultConfig::StrangeEnum::kX, "X"},
                      {DefaultConfig::StrangeEnum::kY, "Y"},
                      {DefaultConfig::StrangeEnum::kZ, "Z"}});
  config::subconfig(config.sub_config, "sub_config", "sub_ns");
  config::subconfig(config.sub_sub_config, "sub_sub_config", "sub_sub_ns");

  config::checkGT(config.i, 0, "i");
  config::checkGE(config.f, 0.f, "f");
  config::checkLT(config.d, 4.0, "d");
  config::checkLE(config.u8, uint8_t(5), "u8");
  config::checkEQ(config.s, std::string("test string"), "s");
  config::checkNE(config.b, false, "b");
  config::checkCondition(config.vec.size() == 3, "Param 'vec' must b of size '3'");
  config::checkInRange(config.d, 0.0, 500.0, "d");
}

void expextDefaultValues(const DefaultConfig& config) {
  EXPECT_EQ(config.i, 1);
  EXPECT_EQ(config.f, 2.1f);
  EXPECT_EQ(config.d, 3.2);
  EXPECT_EQ(config.b, true);
  EXPECT_EQ(config.u8, 4);
  EXPECT_EQ(config.s, "test string");
  EXPECT_EQ(config.vec, std::vector<int>({1, 2, 3}));
  const std::map<std::string, int> map({{"a", 1}, {"b", 2}, {"c", 3}});
  EXPECT_EQ(config.map, map);
  EXPECT_EQ(config.set, std::set<float>({1.1f, 2.2, 3.3f}));
  const auto mat = Eigen::Matrix<double, 3, 3>::Identity();
  EXPECT_EQ(config.mat, mat);
  EXPECT_EQ(config.my_enum, DefaultConfig::Enum::kA);
  EXPECT_EQ(config.my_strange_enum, DefaultConfig::StrangeEnum::kX);
  EXPECT_EQ(config.sub_config.i, 1);
  EXPECT_EQ(config.sub_config.sub_sub_config.i, 1);
  EXPECT_EQ(config.sub_sub_config.i, 1);
}

void expectModifiedValues(const DefaultConfig& config) {
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
  EXPECT_EQ(config.sub_config.i, 2);
  EXPECT_EQ(config.sub_config.sub_sub_config.i, 3);
  EXPECT_EQ(config.sub_sub_config.i, 4);
}

}  // namespace config::test
