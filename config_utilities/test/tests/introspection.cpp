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

#include <gtest/gtest.h>

#include "config_utilities/parsing/commandline.h"
#include "config_utilities/settings.h"
#include "config_utilities/test/cli_args.h"
#include "config_utilities/test/utils.h"

namespace config::test {

TEST(Introspection, setupFromParser) {
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
  EXPECT_EQ(config::Settings().introspection.output, "introspection_results");
}

}  // namespace config::test
