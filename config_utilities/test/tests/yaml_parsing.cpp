#include <gtest/gtest.h>

#include "config_utilities/traits.h"

namespace config::test {

struct Config {};

void declare_config(Config& config) {}

struct NotAConfig {};

TEST(YamlParsing, IsConfig) {
  Config config;
  NotAConfig not_a_config;
  EXPECT_EQ(isConfig<Config>(), true);
  EXPECT_EQ(isConfig<NotAConfig>(), false);
  EXPECT_EQ(isConfig(config), true);
  EXPECT_EQ(isConfig(not_a_config), false);
}

}  // namespace config::test
