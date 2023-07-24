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

  std::string has_leading_whitespace = "   hello world";
  EXPECT_EQ(internal::pruneLeadingWhitespace(has_leading_whitespace), has_no_whitespace);
  EXPECT_EQ(internal::pruneLeadingWhitespace(has_no_whitespace), has_no_whitespace);
  EXPECT_EQ(internal::pruneLeadingWhitespace(empty), empty);
  EXPECT_EQ(internal::pruneLeadingWhitespace(all_whitespace), empty);
}

TEST(StringUtils, wrapString) {
  std::string str = "A very long string that needs to be wrapped to fit into a given width.";
  std::string wrapped = internal::wrapString(str, 12);
  std::string expected = R"""(A very long
string that
needs to be
wrapped to f
it into a gi
ven width.)""";
  EXPECT_EQ(wrapped, expected);

  std::string wrapped_with_indent = internal::wrapString(str, 12, 2);
  expected = R"""(  A very lon
  g string t
  hat needs
  to be wrap
  ped to fit
  into a giv
  en width.)""";
  EXPECT_EQ(wrapped_with_indent, expected);

  std::string wrapped_with_indent_no_first_line = internal::wrapString(str, 12, 2, false);
  expected = R"""(A very long
  string tha
  t needs to
  be wrapped
  to fit int
  o a given
  width.)""";
  EXPECT_EQ(wrapped_with_indent_no_first_line, expected);

  str = "    A very long string with leading and trailing whitespaces.      ";
  wrapped = internal::wrapString(str, 12);
  expected = R"""(    A very l
ong string w
ith leading
and trailing
whitespaces.)""";
  EXPECT_EQ(wrapped, expected);

  wrapped_with_indent = internal::wrapString(str, 12, 2);
  expected = R"""(      A very
  long strin
  g with lea
  ding and t
  railing wh
  itespaces.)""";
  EXPECT_EQ(wrapped_with_indent, expected);

  wrapped_with_indent_no_first_line = internal::wrapString(str, 12, 2, false);
  expected = R"""(    A very l
  ong string
  with leadi
  ng and tra
  iling whit
  espaces.)""";
  EXPECT_EQ(wrapped_with_indent_no_first_line, expected);
}

}  // namespace config::test
