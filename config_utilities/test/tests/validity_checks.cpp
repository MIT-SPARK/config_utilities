#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/validation.h"

namespace config::test {

// Tools.
int numChecks(const DefaultConfig& config) {
  internal::MetaData data = internal::Visitor::getChecks(config);
  int num_checks = 0;
  data.performOnAll([&num_checks](const internal::MetaData& d) { num_checks += d.checks.size(); });
  return num_checks;
}

int numFailedChecks(const DefaultConfig& config) {
  internal::MetaData data = internal::Visitor::getChecks(config);
  int num_checks = 0;
  data.performOnAll([&num_checks](const internal::MetaData& d) {
    for (const auto& check : d.checks) {
      if (!check->valid()) {
        num_checks++;
      }
    }
  });
  return num_checks;
}

bool throwsException(const DefaultConfig& config) {
  try {
    checkValid(config);
  } catch (...) {
    return true;
  }
  return false;
}

template <bool LOpen, bool UOpen>
struct RangeConfig {
  static constexpr bool lower_open = LOpen;
  static constexpr bool upper_open = UOpen;

  RangeConfig() { value.reset(new int32_t(5)); }

  explicit RangeConfig(const std::shared_ptr<int32_t>& value) : value(value) {}

  virtual ~RangeConfig() = default;

  std::shared_ptr<int32_t> value;
};

using LowerOpen = RangeConfig<true, false>;
using UpperOpen = RangeConfig<false, true>;
using BothOpen = RangeConfig<true, true>;
using Closed = RangeConfig<false, false>;

template <bool LOpen, bool UOpen>
void declare_config(RangeConfig<LOpen, UOpen>& config) {
  checkInRange(*config.value, 0, 10, "value in range", !config.lower_open, !config.upper_open);
}

struct CustomIndex {
  CustomIndex() = default;
  CustomIndex(std::string _key, int _index) : key(std::move(_key)), index(_index) {}

  bool operator==(const CustomIndex& other) const { return key == other.key && index == other.index; }

  std::string key;
  int index;
};
std::ostream& operator<<(std::ostream& os, const CustomIndex& index) {
  return os << "<" << index.key << "," << index.index << ">";
}

// Tests.
TEST(ValidityChecks, rangeBoundaries) {
  // declare three different ranges with the same underlying value
  std::shared_ptr<int32_t> value(new int32_t(5));
  LowerOpen lower_open(value);
  UpperOpen upper_open(value);
  BothOpen both_open(value);
  Closed closed(value);

  // all four configs start valid
  EXPECT_EQ(isValid(lower_open), true);
  EXPECT_EQ(isValid(upper_open), true);
  EXPECT_EQ(isValid(both_open), true);
  EXPECT_EQ(isValid(closed), true);

  // hit lower bound with value: open ranges fail
  *value = 0;
  EXPECT_EQ(isValid(lower_open), false);
  EXPECT_EQ(isValid(upper_open), true);
  EXPECT_EQ(isValid(both_open), false);
  EXPECT_EQ(isValid(closed), true);

  // hit upper bound with value: open ranges fail
  *value = 10;
  EXPECT_EQ(isValid(lower_open), true);
  EXPECT_EQ(isValid(upper_open), false);
  EXPECT_EQ(isValid(both_open), false);
  EXPECT_EQ(isValid(closed), true);

  // move outside range: everything fails
  *value = 20;
  EXPECT_EQ(isValid(lower_open), false);
  EXPECT_EQ(isValid(upper_open), false);
  EXPECT_EQ(isValid(both_open), false);
  EXPECT_EQ(isValid(closed), false);
}

TEST(ValidityChecks, isOneOf) {
  internal::CheckIsOneOf check(1, {1, 2, 3}, "test");
  EXPECT_TRUE(check.valid());
  EXPECT_EQ(check.name(), "test");

  internal::CheckIsOneOf failed_check(1, {2, 3}, "test");
  EXPECT_FALSE(failed_check.valid());
  EXPECT_EQ(failed_check.message(), "param must be one of ['2', '3'] (is: '1')");

  internal::CheckIsOneOf empty_check(1, {}, "test");
  EXPECT_FALSE(empty_check.valid());
  EXPECT_EQ(empty_check.message(), "param must be one of [] (is: '1')");

  internal::CheckIsOneOf single_check = internal::CheckIsOneOf(0, {1}, "test");
  EXPECT_EQ(single_check.message(), "param must be one of ['1'] (is: '0')");

  std::vector<CustomIndex> candidates;
  candidates.emplace_back("a", 1);
  candidates.emplace_back("a", 2);
  candidates.emplace_back("b", 1);
  internal::CheckIsOneOf custom_check(CustomIndex("a", 1), candidates, "test");
  EXPECT_TRUE(custom_check.valid());

  internal::CheckIsOneOf custom_failed_check(CustomIndex("b", 2), candidates, "test");
  EXPECT_FALSE(custom_failed_check.valid());
  EXPECT_EQ(custom_failed_check.message(), "param must be one of ['<a,1>', '<a,2>', '<b,1>'] (is: '<b,2>')");
}

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
  config.sub_sub_config.i = -1;
  EXPECT_EQ(isValid(config), false);

  config = DefaultConfig();
  config.sub_config.sub_sub_config.i = -1;
  EXPECT_EQ(isValid(config), false);
}

TEST(ValidityChecks, numChecks) {
  DefaultConfig config;
  EXPECT_EQ(numChecks(config), 11);
  EXPECT_EQ(numFailedChecks(config), 0);

  config.i = -1;
  EXPECT_EQ(numFailedChecks(config), 1);

  config.f = -1.f;
  EXPECT_EQ(numFailedChecks(config), 2);

  config.d = 100.0;
  EXPECT_EQ(numFailedChecks(config), 3);

  config.u8 = 255;
  EXPECT_EQ(numFailedChecks(config), 4);

  config.s = "";
  EXPECT_EQ(numFailedChecks(config), 5);

  config.vec = {1, 2};
  EXPECT_EQ(numFailedChecks(config), 6);

  config.b = false;
  EXPECT_EQ(numFailedChecks(config), 7);

  config.d = 1000.0;
  EXPECT_EQ(numFailedChecks(config), 8);

  config.sub_config.i = -1;
  EXPECT_EQ(numFailedChecks(config), 9);

  config.sub_sub_config.i = -1;
  EXPECT_EQ(numFailedChecks(config), 10);

  config.sub_config.sub_sub_config.i = -1;
  EXPECT_EQ(numFailedChecks(config), 11);
  EXPECT_EQ(numChecks(config), 11);
}

TEST(ValidityChecks, checkValid) {
  DefaultConfig config;
  EXPECT_FALSE(throwsException(config));

  config.i = -1;
  EXPECT_TRUE(throwsException(config));
}

}  // namespace config::test
