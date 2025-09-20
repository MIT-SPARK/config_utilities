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

#include "config_utilities/internal/introspection.h"

#include <gtest/gtest.h>

#include "config_utilities/parsing/commandline.h"
#include "config_utilities/settings.h"
#include "config_utilities/test/cli_args.h"
#include "config_utilities/test/introspection_utils.h"
#include "config_utilities/test/utils.h"

namespace config::test {

using Event = internal::Introspection::Event;
using By = internal::Introspection::By;
using Intro = internal::Introspection;

TEST(Introspection, invokeFromParser) {
  reset();
  CliArgs cli_args(std::vector<std::string>{"some_command",
                                            "--config-utilities-file",
                                            "resources/foo.yaml",
                                            "--config-utilities-file",
                                            "resources/bar.yaml",
                                            "--config-utilities-introspect",
                                            "/path/to/output"});

  // With specified output directory
  auto args = cli_args.get();
  auto node = internal::loadFromArguments(args.argc, args.argv, true);
  EXPECT_EQ(args.get_cmd(), "some_command");
  EXPECT_EQ(config::Settings().introspection.output, "/path/to/output");

  // Default output directory.
  cli_args = CliArgs(std::vector<std::string>{"some_command",
                                              "--config-utilities-file",
                                              "resources/foo.yaml",
                                              "--config-utilities-file",
                                              "resources/bar.yaml",
                                              "--config-utilities-introspect"});
  args = cli_args.get();
  node = internal::loadFromArguments(args.argc, args.argv, true);
  EXPECT_EQ(args.get_cmd(), "some_command");
  EXPECT_EQ(config::Settings().introspection.output, intro_dir);
}

TEST(Introspection, renderStateFromHistory) {
  reset();

  // Setup 1.
  YAML::Node node1 = YAML::Load("{a: 5, b: [1, 2], c: {d: 10, e: 20}}");
  Intro::logMerge(node1, node1, By::file("file0.yaml"));

  // Merge 2.
  YAML::Node in = YAML::Load("{a: 7, c: {e: {f: blipp}}}");
  YAML::Node node2 = YAML::Clone(node1);
  internal::mergeYamlNodes(node2, in, internal::MergeMode::APPEND);
  Intro::logMerge(node2, in, By::file("file1.yaml"));

  // Merge 3.
  in = YAML::Load("{b: [3], c: {d: 15}}");
  YAML::Node node3 = YAML::Clone(node2);
  internal::mergeYamlNodes(node3, in, internal::MergeMode::APPEND);
  Intro::logMerge(node3, in, By::file("file2.yaml"));

  // Overwriting sets: list to map & upstream scalars.
  in = YAML::Load("{b: {foo: bar}, c: 123}");
  YAML::Node node4 = YAML::Clone(node3);
  internal::mergeYamlNodes(node4, in, internal::MergeMode::REPLACE);
  Intro::logMerge(node4, in, By::file("file3.yaml"));

  // Clear.
  YAML::Node node5 = YAML::Node();
  Intro::logClear(By::programmatic("test clear"));

  // Set again with new values.
  in = YAML::Load("{x: 42, b: [{nested: true}, {nested: false}], c: {sub: \"<!emulate | subst>\"}}");
  YAML::Node node6 = YAML::Clone(node5);
  internal::mergeYamlNodes(node6, in, internal::MergeMode::APPEND);
  Intro::logMerge(node6, in, By::file("file4.yaml"));

  // Emulate substitution resolution.
  YAML::Node node7 = YAML::Clone(node6);
  node7["c"]["sub"] = "substituted_value";
  Intro::logDiff(node6, node7, By::substitution("emulated substitution"));

  // Render out the history after the fact.
  auto rendered0 = Intro::instance().data().toYaml(0);
  YAML::Node empty;
  expectEqual(empty, rendered0);

  auto rendered1 = Intro::instance().data().toYaml(1);
  expectEqual(node1, rendered1);

  auto rendered2 = Intro::instance().data().toYaml(2);
  expectEqual(node2, rendered2);

  auto rendered3 = Intro::instance().data().toYaml(3);
  expectEqual(node3, rendered3);

  auto rendered4 = Intro::instance().data().toYaml(4);
  expectEqual(node4, rendered4);

  auto rendered5 = Intro::instance().data().toYaml(5);
  expectEqual(node5, rendered5);

  auto rendered6 = Intro::instance().data().toYaml(6);
  expectEqual(node6, rendered6);

  auto rendered7 = Intro::instance().data().toYaml(7);
  expectEqual(node7, rendered7);
}

}  // namespace config::test
