#include "config_utilities/globals.h"

#include <gtest/gtest.h>

#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/validation.h"

namespace config::test {

TEST(Globals, PrintValidChecks) {
  Settings().restoreDefaults();
  Settings().store_valid_configs = true;
  internal::Globals::instance().valid_configs.clear();

  DefaultConfig config;
  checkValid(config);
  checkValid(config);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 2);

  config.i = 123;
  config.f = 456.7f;
  checkValid(config);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 3);

  auto logger = TestLogger::create();
  config.i = -1;
  checkValid(config);
  EXPECT_EQ(logger->messages().back().first, internal::Severity::kFatal);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 3);

  std::string msg = printAllValidConfigs();
  const std::string expected = R"""(================================ DefaultConfig =================================
i [m]:                        1 (default)
f [s]:                        2.1 (default)
d [m/s]:                      3.2 (default)
b:                            true (default)
u8:                           4 (default)
s:                            test string (default)
vec [frames]:                 [1, 2, 3] (default)
map:                          {a: 1, b: 2, c: 3} (default)
set:                          [1.1, 2.2, 3.3] (default)
mat:                          [[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]] (default)
my_enum:                      A (default)
my_strange_enum:              X (default)
sub_config [SubConfig] (default):
   i:                         1 (default)
   sub_sub_config [SubSubConfig] (default):
      i:                      1 (default)
sub_sub_config [SubSubConfig] (default):
   i:                         1 (default)
================================ DefaultConfig =================================
i [m]:                        1 (default)
f [s]:                        2.1 (default)
d [m/s]:                      3.2 (default)
b:                            true (default)
u8:                           4 (default)
s:                            test string (default)
vec [frames]:                 [1, 2, 3] (default)
map:                          {a: 1, b: 2, c: 3} (default)
set:                          [1.1, 2.2, 3.3] (default)
mat:                          [[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]] (default)
my_enum:                      A (default)
my_strange_enum:              X (default)
sub_config [SubConfig] (default):
   i:                         1 (default)
   sub_sub_config [SubSubConfig] (default):
      i:                      1 (default)
sub_sub_config [SubSubConfig] (default):
   i:                         1 (default)
================================ DefaultConfig =================================
i [m]:                        123
f [s]:                        456.7
d [m/s]:                      3.2 (default)
b:                            true (default)
u8:                           4 (default)
s:                            test string (default)
vec [frames]:                 [1, 2, 3] (default)
map:                          {a: 1, b: 2, c: 3} (default)
set:                          [1.1, 2.2, 3.3] (default)
mat:                          [[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]] (default)
my_enum:                      A (default)
my_strange_enum:              X (default)
sub_config [SubConfig] (default):
   i:                         1 (default)
   sub_sub_config [SubSubConfig] (default):
      i:                      1 (default)
sub_sub_config [SubSubConfig] (default):
   i:                         1 (default)
================================================================================)""";
  EXPECT_EQ(msg, expected);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 3);

  msg = printAllValidConfigs(true);
  EXPECT_EQ(msg, expected);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 0);
}

}  // namespace config::test
