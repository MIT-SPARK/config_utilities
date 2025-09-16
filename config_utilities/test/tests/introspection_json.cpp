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

#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "config_utilities/internal/introspection.h"
#include "config_utilities/parsing/commandline.h"
#include "config_utilities/settings.h"
#include "config_utilities/test/cli_args.h"
#include "config_utilities/test/utils.h"

namespace config::test {

const std::string intro_dir = "config_introspection_output";

bool loadData(nlohmann::json& j) {
  const std::string intro_file = "config_introspection_output/data.json";
  if (!std::filesystem::exists(intro_file)) {
    return false;
  }
  std::ifstream(intro_file) >> j;
  return true;
}

TEST(Introspection, logCLIFile) {
  internal::Introspection::instance().clear();
  CliArgs cli_args(std::vector<std::string>{"some_command", "--config-utilities-file", "resources/foo.yaml@foo", "-i"});
  auto args = cli_args.get();
  auto node = internal::loadFromArguments(args.argc, args.argv, true);
  nlohmann::json j;
  EXPECT_TRUE(loadData(j));
  const nlohmann::json expected = R"({
  "data": {
    "foo/a": [
      {
        "by": "f0",
        "type": "s",
        "val": "5.0"
      }
    ],
    "foo/b": [
      {
        "by": "f0",
        "type": "s",
        "val": "[1, 2, 3]"
      }
    ],
    "foo/c": [
      {
        "by": "f0",
        "type": "s",
        "val": "hello"
      }
    ]
  },
  "sources": {
    "a": [
      "config-utilities-introspect"
    ],
    "f": [
      "/home/lukas/khronos_ws/build/config_utilities/test/resources/foo.yaml@foo"
    ]
  }
})"_json;
  EXPECT_EQ(j, expected);
}

TEST(Introspection, logCLIYaml) {
  internal::Introspection::instance().clear();
  CliArgs cli_args(std::vector<std::string>{
      "some_command", "--config-utilities-yaml", "foo: {a: 5.0,  b: [1, 2, 3],  sub_ns: {c: hello}}", "-i"});
  auto args = cli_args.get();
  auto node = internal::loadFromArguments(args.argc, args.argv, true);
  nlohmann::json j;
  EXPECT_TRUE(loadData(j));
  std::cout << j.dump(2) << std::endl;
}

}  // namespace config::test
