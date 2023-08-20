/** -----------------------------------------------------------------------------
 * Copyright (c) 2023 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
 * AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2023
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

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
  std::string str = "short.";
  std::string wrapped = internal::wrapString(str, 12);
  EXPECT_EQ(wrapped, str);

  str = "short.        ";
  wrapped = internal::wrapString(str, 12);
  EXPECT_EQ(wrapped, "short.");

  str = "A very long string that needs to be wrapped to fit into a given width.";
  wrapped = internal::wrapString(str, 12);
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
