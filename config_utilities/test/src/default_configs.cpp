
#include "config_utilities/test/default_configs.h"

#include "config_utilities/config.h"

namespace config::test {

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

  config::checkGT(config.i, 0, "i");
  config::checkGE(config.f, 0.f, "f");
  config::checkLT(config.d, 4.0, "d");
  config::checkLE(config.u8, uint8_t(5), "u8");
  config::checkEQ(config.s, std::string("test string"), "s");
  config::checkNE(config.b, false, "b");
  config::checkCondition(config.vec.size() == 3, "Param 'vec' must b of size '3'");
  config::checkInRange(config.d, 0.0, 500.0, "d");
}

}  // namespace config::test
