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

#include "config_utilities/config.h"
#include "config_utilities/formatting/asl.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/logging/log_to_stdout.h"
#include "config_utilities/printing.h"
#include "config_utilities/virtual_config.h"

namespace config::test {

struct SimpleConfig {
  double a = 1.0;
  int b = 2;
  std::string c = "3";
};

void declare_config(SimpleConfig& config) {
  name("SimpleConfig");
  field(config.a, "a");
  field(config.b, "b");
  field(config.c, "c");
  check(config.b, GT, 2, "b");
}

bool operator==(const SimpleConfig& lhs, const SimpleConfig& rhs) {
  return lhs.a == rhs.a && lhs.b == rhs.b && lhs.c == rhs.c;
}

struct ComposedConfig {
  SimpleConfig simple;
  double other = 0.0;
};

void declare_config(ComposedConfig& config) {
  name("ComposedConfig");
  field(config.simple, "simple");
  field(config.other, "other");
  check(config.other, GT, 1.0, "other");
}

void PrintTo(const SimpleConfig& config, std::ostream* os) { *os << toString(config); }
void PrintTo(const ComposedConfig& config, std::ostream* os) { *os << toString(config); }

TEST(MissingFields, ParseMissing) {
  const std::string contents = "{simple: {a: 2.0, b: 3}}";
  const auto node = YAML::Load(contents);
  ComposedConfig result;
  const auto data = config::internal::Visitor::setValues(result, node);

  EXPECT_TRUE(data.hasMissing());
  const auto message = internal::Formatter::formatMissing(data);
  const std::string expected_message = R"msg( 'ComposedConfig':
================================ ComposedConfig ================================
Warning: Missing field 'other'!
Warning: Missing field 'simple.c'!
================================================================================)msg";
  EXPECT_EQ(message, expected_message);
}

TEST(MissingFields, ParseNoMissing) {
  const std::string contents = "{a: 2.0, b: 3, c: '4'}";
  const auto node = YAML::Load(contents);
  SimpleConfig result;
  const auto data = config::internal::Visitor::setValues(result, node);

  SimpleConfig expected{2.0, 3, "4"};
  EXPECT_EQ(expected, result);
  EXPECT_FALSE(data.hasMissing());
}

}  // namespace config::test
