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

#include "config_utilities/substitutions.h"

#include <gtest/gtest.h>

#include "config_utilities/test/utils.h"

namespace config::test {

namespace {

inline YAML::Node doResolve(const YAML::Node& orig,
                            const std::map<std::string, std::string>& args = {},
                            bool strict = true) {
  auto result = YAML::Clone(orig);
  ParserContext context;
  context.vars = args;
  resolveSubstitutions(result, context, strict);
  return result;
}

}  // namespace

TEST(Substitutions, clearLeftoverTags) {
  const auto node = YAML::Load(R"""(
root:
  children:
    - {a: !append [4]}
    - {c: !replace [0]}
  map_with_tags:
    !append tagged_key:
      foo:
        - a: {b: !replace 2}
    other:
      bar: 42
      foo: 7
)""");

  auto result = doResolve(node);
  const auto expected = YAML::Load(R"""(
root:
  children:
    - {a: [4]}
    - {c: [0]}
  map_with_tags:
    tagged_key:
      foo: [{a: {b: 2}}]
    other: {bar: 42, foo: 7}
)""");
  expectEqual(result, expected);
}

TEST(Substitutions, resolveEnv) {
  {  // check that we don't try to pass non-scalars to getenv
    const auto node = YAML::Load("root: $<env | [1, 2, 3]>");
    const auto result = doResolve(node, {}, false);
    const auto expected = YAML::Load("root: $<env | [1, 2, 3]>");
  }

  auto unset = std::getenv("/some/random/env/variable");
  if (unset) {
    FAIL() << "environment variable '/some/random/env/variable' is set";
  } else {
    try {
      const auto node = YAML::Load("root: $<env | /some/random/env/variable>");
      const auto result = doResolve(node);
      FAIL();
    } catch (const std::runtime_error& e) {
      std::string msg(e.what());
      EXPECT_NE(msg.find("Invalid substitution in node"), std::string::npos);
    }
  }

  auto set = std::getenv("HOME");
  if (!set) {
    FAIL() << "required environment variable 'HOME' not set";
  } else {
    const auto node = YAML::Load("root: $<env | HOME>");
    const auto result = doResolve(node);
    const auto expected = YAML::Load("root: " + std::string(set));
    expectEqual(result, expected);
  }
}

TEST(Substitutions, interpolateFlatSubstitutions) {
  std::map<std::string, std::string> args{{"foo", "a"}, {"bar", "b"}};
  const auto node = YAML::Load("root: other/$<var | foo>/test a/$<var | bar>/c");
  const auto result = doResolve(node, args);
  const auto expected = YAML::Load("root: other/a/test a/b/c");
  expectEqual(result, expected);
}

TEST(Substitutions, nestedSubstitutions) {
  {  // nested subs without terminal suffixes and prefixes
    std::map<std::string, std::string> args{{"foo", "a"}, {"bar", "b"}, {"a", "foo"}};
    const auto node = YAML::Load("root: other/$<var | $<var | foo>>/test a/$<var | bar>/c");
    const auto result = doResolve(node, args);
    const auto expected = YAML::Load("root: other/foo/test a/b/c");
    expectEqual(result, expected);
  }

  {  // nested subs with terminal suffixes and prefixes
    std::map<std::string, std::string> args{{"foo", "a"}, {"bar", "b"}, {"baa", "foo"}};
    const auto node = YAML::Load("root: \"other/$<var b$<var foo>a>/test\na/$<var $<var $<var bar>aa>>/c\"");
    const auto result = doResolve(node, args);
    const auto expected = YAML::Load("root: \"other/foo/test\na/a/c\"");
    expectEqual(result, expected);
  }
}

}  // namespace config::test
