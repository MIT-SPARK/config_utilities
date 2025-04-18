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

#include "config_utilities/parsing/commandline.h"

#include <gtest/gtest.h>

#include "config_utilities/test/utils.h"

namespace config::test {
namespace {

struct CliArgs {
  struct Args {
    int argc;
    char** argv;

    std::string get_cmd() const {
      std::stringstream ss;
      for (int i = 0; i < argc; ++i) {
        ss << argv[i];
        if (i < argc - 1) {
          ss << " ";
        }
      }

      return ss.str();
    }
  };

  explicit CliArgs(const std::vector<std::string>& args) : original_args(args) {
    for (auto& str : original_args) {
      arg_pointers.push_back(str.data());
    }
  }

  Args get() { return {static_cast<int>(arg_pointers.size()), arg_pointers.data()}; }

  std::vector<std::string> original_args;
  std::vector<char*> arg_pointers;
};

}  // namespace

TEST(Commandline, noInputArgs) {
  CliArgs cli_args(std::vector<std::string>{"some_command"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const YAML::Node expected;
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command");
}

TEST(Commandline, invalidFlags) {
  CliArgs cli_args(std::vector<std::string>{"some_command", "--config-utilities-file", "--config-utilities-yaml"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const YAML::Node expected;
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command --config-utilities-file --config-utilities-yaml");
}

TEST(Commandline, missingFile) {
  CliArgs cli_args(std::vector<std::string>{"some_command", "--config-utilities-file", "resources/missing.yaml"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const YAML::Node expected;
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command");
}

TEST(Commandline, invalidFile) {
  CliArgs cli_args(std::vector<std::string>{"some_command", "--config-utilities-file", "resources/invalid.yaml"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const YAML::Node expected;
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command");
}

TEST(Commandline, vaildFiles) {
  CliArgs cli_args(std::vector<std::string>{"some_command",
                                            "--config-utilities-file",
                                            "resources/foo.yaml",
                                            "--config-utilities-file",
                                            "resources/bar.yaml"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const auto expected = YAML::Load(R"yaml(
a: 6.0
b: [1, 2, 3, 4]
c: hello
d: world!
  )yaml");
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command");
}

TEST(Commandline, fileOrderingCorrect) {
  CliArgs cli_args(std::vector<std::string>{"some_command",
                                            "--config-utilities-file",
                                            "resources/bar.yaml",
                                            "--config-utilities-file",
                                            "resources/foo.yaml"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, false);
  const auto expected = YAML::Load(R"yaml(
a: 5.0
b: [4, 1, 2, 3]
c: hello
d: world!
  )yaml");
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(),
            "some_command --config-utilities-file resources/bar.yaml --config-utilities-file resources/foo.yaml");
}

TEST(Commandline, mixedYamlAndFiles) {
  // note that ROS can't escape correctly so args are broken by space
  CliArgs cli_args(std::vector<std::string>{"some_command",
                                            "--config-utilities-file",
                                            "resources/foo.yaml",
                                            "--config-utilities-yaml",
                                            "{a:",
                                            "6.0,",
                                            "b:",
                                            "[4],",
                                            "d:",
                                            "world!}"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const auto expected = YAML::Load(R"yaml(
a: 6.0
b: [1, 2, 3, 4]
c: hello
d: world!
  )yaml");
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command");
}

TEST(Commandline, InvalidYaml) {
  // note that ROS can't escape correctly so args are broken by space
  CliArgs cli_args(std::vector<std::string>{
      "some_command", "--config-utilities-file", "resources/foo.yaml", "--config-utilities-yaml", "{a:", "6.0,"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const auto expected = YAML::Load(R"yaml(
a: 5.0
b: [1, 2, 3]
c: hello
  )yaml");
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command");
}

TEST(Commandline, NamespacedFile) {
  // note that ROS can't escape correctly so args are broken by space
  // NOTE(nathan) adds intermediate flags to also check arg removal
  // NOTE(nathan) order should be preserved: bar.yaml should override a for inline yaml
  CliArgs cli_args(std::vector<std::string>{"some_command",
                                            "--config-utilities-file",
                                            "resources/foo.yaml@foo/other",
                                            "--verbose=true",
                                            "--config-utilities-yaml",
                                            "{c:",
                                            "6.0, a: 7.0}",
                                            "--some-flag",
                                            "--config-utilities-file",
                                            "resources/bar.yaml@",
                                            "--ros-args",
                                            "-r",
                                            "other_arg:=something"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const auto expected = YAML::Load(R"yaml(
foo:
  other:
    a: 5.0
    b: [1, 2, 3]
    c: hello
c: 6.0
a: 6.0
b: [4]
d: world!
  )yaml");
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command --verbose=true --some-flag --ros-args -r other_arg:=something");
}

TEST(Commandline, NegativeValue) {
  // Checks that we correctly don't detect negative values as flags
  CliArgs cli_args(std::vector<std::string>{"some_command", "--config-utilities-yaml", "{c:", "6.0, a:", "-7.0}"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const auto expected = YAML::Load(R"yaml({c: 6.0, a: -7.0})yaml");
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command");
}

TEST(Commandline, ShortOpt) {
  // Checks that we correctly don't detect negative values as flags
  CliArgs cli_args(std::vector<std::string>{"some_command",
                                            "--config-utilities-yaml",
                                            "{c:",
                                            "6.0, a:",
                                            "-7.0}",
                                            "-h",
                                            "--config-utilities-yaml",
                                            "{c:",
                                            "-h,",
                                            "a:",
                                            "9.0}"});
  auto args = cli_args.get();
  const auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const auto expected = YAML::Load(R"yaml({c: '-h', a: 9.0})yaml");
  expectEqual(expected, node);
  EXPECT_EQ(args.get_cmd(), "some_command -h");
}

}  // namespace config::test
