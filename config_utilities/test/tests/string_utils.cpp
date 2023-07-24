#include "config_utilities/internal/string_utils.h"

#include <gtest/gtest.h>

namespace config::test {

TEST(StringUtils, pruneWhitespace) {
  std::string has_whitespace = "hello world   ";
  std::string has_no_whitespace = "hello world";
  EXPECT_EQ(internal::pruneTrailingWhitespace(has_whitespace), has_no_whitespace);
  EXPECT_EQ(internal::pruneTrailingWhitespace(has_no_whitespace), has_no_whitespace);

  std::string empty = "";
  std::string all_whitespace = "             ";
  EXPECT_EQ(internal::pruneTrailingWhitespace(all_whitespace), empty);
  EXPECT_EQ(internal::pruneTrailingWhitespace(empty), empty);
}

}  // namespace config::test
