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

#include <iostream>

#include <config_utilities/config.h>
#include <config_utilities/external_registry.h>
#include <config_utilities/factory.h>
#include <config_utilities/logging/log_to_stdout.h>

#include "config_utilities/test/external_types.h"

namespace external {

struct ExternalTalker : config::test::Talker {
  std::string talk() const override { return "external"; }
  inline static const auto reg = config::Registration<config::test::Talker, ExternalTalker>("external");
};

struct FancyTalker : config::test::Talker {
  struct Config {
    std::string prefix = "* ";
    std::string suffix = " *";
  } const config;

  explicit FancyTalker(const Config& config) : config(config) {}

  std::string talk() const override { return config.prefix + "derived" + config.suffix; }

  inline static const auto reg = config::RegistrationWithConfig<config::test::Talker, FancyTalker, Config>("fancy");
};

void declare_config(FancyTalker::Config& config) {
  config::name("FancyTalker::Config");
  config::field(config.prefix, "prefix");
  config::field(config.suffix, "suffix");
}

struct ExternalLogger : config::internal::Logger {
  void logImpl(const config::internal::Severity, const std::string&) { throw std::runtime_error("External logger!"); }
  inline static const auto reg = config::Registration<config::internal::Logger, ExternalLogger>("external_logger");
};

}  // namespace external
