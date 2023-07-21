#include "config_utilities/validity_checks.h"

#include <gtest/gtest.h>

#include "config_utilities/test/default_configs.h"

namespace config::test {

// Tools.
int numErrors(const DefaultConfig& config) {
  internal::MetaData data = internal::Visitor::getChecks(config);
  int num_errors = 0;
  data.performOnAll([&num_errors](const internal::MetaData& d) { num_errors += d.errors.size(); });
  return num_errors;
}

bool throwsException(const DefaultConfig& config) {
  try {
    checkValid(config);
  } catch (...) {
    return true;
  }
  return false;
}

// Tests.
TEST(ValidityChecks, isValid) {
  DefaultConfig config;
  EXPECT_EQ(isValid(config), true);

  config.i = -1;
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.f = -1.f;
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.d = 100.0;
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.u8 = 255;
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.s = "";
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.vec = {1, 2};
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.b = false;
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.d = -1;
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.sub_config.i = -1;
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.sub_config.sub_sub_config.i = -1;
  EXPECT_EQ(isValid(config), false);
}

TEST(ValidityChecks, numErrors) {
  DefaultConfig config;
  EXPECT_EQ(numErrors(config), 0);

  config.i = -1;
  EXPECT_EQ(numErrors(config), 1);

  config.f = -1.f;
  EXPECT_EQ(numErrors(config), 2);

  config.d = 100.0;
  EXPECT_EQ(numErrors(config), 3);

  config.u8 = 255;
  EXPECT_EQ(numErrors(config), 4);

  config.s = "";
  EXPECT_EQ(numErrors(config), 5);

  config.vec = {1, 2};
  EXPECT_EQ(numErrors(config), 6);

  config.b = false;
  EXPECT_EQ(numErrors(config), 7);

  config.d = 1000.0;
  EXPECT_EQ(numErrors(config), 8);

  config.sub_config.i = -1;
  EXPECT_EQ(numErrors(config), 9);

  config.sub_config.sub_sub_config.i = -1;
  EXPECT_EQ(numErrors(config), 10);
}

TEST(ValidityChecks, checkValid) {
  DefaultConfig config;
  EXPECT_FALSE(throwsException(config));

  config.i = -1;
  EXPECT_TRUE(throwsException(config));
}

}  // namespace config::test
