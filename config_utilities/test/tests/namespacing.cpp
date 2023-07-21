#include <gtest/gtest.h>

#include "config_utilities/internal/string_utils.h"

namespace config::test {

TEST(Namespacing, splitNamespace) {
  std::string ns = "a/b/c";
  const std::vector<std::string> ns_expected = {"a", "b", "c"};
  EXPECT_EQ(internal::splitNamespace(ns), ns_expected);

  ns = "/a/b/c/";
  EXPECT_EQ(internal::splitNamespace(ns), ns_expected);

  ns = "a/b///c/";
  EXPECT_EQ(internal::splitNamespace(ns), ns_expected);
}

TEST(Namespacing, joinNamespace) {
  std::vector<std::string> ns = {"a", "b", "c"};
  const std::string ns_expected = "a/b/c";
  EXPECT_EQ(internal::joinNamespace(ns), ns_expected);

  std::string ns1 = "a";
  std::string ns2 = "b/c";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), ns_expected);

  ns1 = "/a";
  ns2 = "/b/c";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), ns_expected);

  ns1 = "a/b///";
  ns2 = "//c/";
  EXPECT_EQ(internal::joinNamespace(ns1, ns2), ns_expected);
}

}  // namespace config::test
