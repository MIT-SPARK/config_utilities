#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "config_utilities/types/eigen_matrix.h"

namespace config::test {

struct SubSubConfig {
  int i = 1;
};

struct SubConfig {
  int i = 1;
  SubSubConfig sub_sub_config;
};

struct DefaultConfig {
  int i = 1;
  float f = 2.1f;
  double d = 3.2;
  bool b = true;
  uint8_t u8 = 4;
  std::string s = "test string";
  std::vector<int> vec = {1, 2, 3};
  std::map<std::string, int> map = {{"a", 1}, {"b", 2}, {"c", 3}};
  std::set<float> set = {1.1f, 2.2, 3.3f};
  Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Identity();
  enum class Enum { kA, kB, kC } my_enum = Enum::kA;
  enum class StrangeEnum : int { kX = 0, kY = 42, kZ = -7 } my_strange_enum = StrangeEnum::kX;
  SubConfig sub_config;
  SubSubConfig sub_sub_config;

  static YAML::Node defaultValues();
  static YAML::Node modifiedValues();
  void expectDefaultValues();
  void expectModifiedValues();
};

void declare_config(SubSubConfig& config);

void declare_config(SubConfig& config);

void declare_config(DefaultConfig& config);

}  // namespace config::test
