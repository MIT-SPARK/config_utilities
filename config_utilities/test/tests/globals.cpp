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

#include "config_utilities/globals.h"

#include <gtest/gtest.h>

#include "config_utilities/parsing/yaml.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/validation.h"

namespace config::test {

TEST(Globals, PrintValidChecks) {
  Settings().restoreDefaults();
  Settings().store_valid_configs = true;
  internal::Globals::instance().valid_configs.clear();

  DefaultConfig config;
  checkValid(config);
  checkValid(config);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 2);

  config.i = 123;
  config.f = 456.7f;
  checkValid(config);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 3);

  auto logger = TestLogger::create();
  config.i = -1;
  checkValid(config);
  EXPECT_EQ(logger->messages().back().first, internal::Severity::kFatal);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 3);

  std::string msg = printAllValidConfigs();
  const std::string expected = R"""(================================ DefaultConfig =================================
i [m]:                        1 (default)
f [s]:                        2.1 (default)
d [m/s]:                      3.2 (default)
b:                            true (default)
u8:                           4 (default)
s:                            test string (default)
vec [frames]:                 [1, 2, 3] (default)
map:                          {a: 1, b: 2, c: 3} (default)
set:                          [1.1, 2.2, 3.3] (default)
mat:                          [[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]] (default)
my_enum:                      A (default)
my_strange_enum:              X (default)
sub_config [SubConfig] (default):
   i:                         1 (default)
   sub_sub_config [SubSubConfig] (default):
      i:                      1 (default)
sub_sub_config [SubSubConfig] (default):
   i:                         1 (default)
================================ DefaultConfig =================================
i [m]:                        1 (default)
f [s]:                        2.1 (default)
d [m/s]:                      3.2 (default)
b:                            true (default)
u8:                           4 (default)
s:                            test string (default)
vec [frames]:                 [1, 2, 3] (default)
map:                          {a: 1, b: 2, c: 3} (default)
set:                          [1.1, 2.2, 3.3] (default)
mat:                          [[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]] (default)
my_enum:                      A (default)
my_strange_enum:              X (default)
sub_config [SubConfig] (default):
   i:                         1 (default)
   sub_sub_config [SubSubConfig] (default):
      i:                      1 (default)
sub_sub_config [SubSubConfig] (default):
   i:                         1 (default)
================================ DefaultConfig =================================
i [m]:                        123
f [s]:                        456.7
d [m/s]:                      3.2 (default)
b:                            true (default)
u8:                           4 (default)
s:                            test string (default)
vec [frames]:                 [1, 2, 3] (default)
map:                          {a: 1, b: 2, c: 3} (default)
set:                          [1.1, 2.2, 3.3] (default)
mat:                          [[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]] (default)
my_enum:                      A (default)
my_strange_enum:              X (default)
sub_config [SubConfig] (default):
   i:                         1 (default)
   sub_sub_config [SubSubConfig] (default):
      i:                      1 (default)
sub_sub_config [SubSubConfig] (default):
   i:                         1 (default)
================================================================================)""";
  EXPECT_EQ(msg, expected);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 3);

  msg = printAllValidConfigs(true);
  EXPECT_EQ(msg, expected);
  EXPECT_EQ(internal::Globals::instance().valid_configs.size(), 0);
}

}  // namespace config::test
