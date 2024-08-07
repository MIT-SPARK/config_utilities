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

namespace external {

struct Foo {
  virtual ~Foo() = default;
  virtual std::string talk() const = 0;
};

struct Bar : Foo {
  std::string talk() const override { return "bar"; }
  inline static const auto reg = config::Registration<Foo, Bar>("bar");
};

struct Derived : Foo {
  std::string talk() const override { return "derived"; }
  inline static const auto reg = config::Registration<Foo, Derived>("derived");
};

struct TestLogger : config::internal::Logger {
  void logImpl(const config::internal::Severity severity, const std::string& message) {
    record.push_back({severity, message});
  }

  std::vector<std::pair<config::internal::Severity, std::string>> record;
  inline static const auto reg = config::Registration<config::internal::Logger, TestLogger>("test_logger");
};

struct FooWithConfig {
  virtual ~FooWithConfig() = default;
  virtual std::string talk() const = 0;
};

struct BarWithConfig : FooWithConfig {
  struct Config {
    size_t repeats = 5;
    std::string message = "bar";
  } const config;

  explicit BarWithConfig(const Config& config) : config(config) {}

  std::string talk() const override {
    std::stringstream ss;
    for (size_t i = 0; i < config.repeats; ++i) {
      ss << config.message;
    }
    return ss.str();
  }
  inline static const auto reg =
      config::RegistrationWithConfig<FooWithConfig, BarWithConfig, Config>("bar_with_config");
};

void declare_config(BarWithConfig::Config& config) {
  config::name("BarWithConfig::Config");
  config::field(config.repeats, "repeats");
  config::field(config.message, "message");
}

struct DerivedWithConfig : FooWithConfig {
  struct Config {
    std::string prefix = "* ";
    std::string suffix = " *";
  } const config;

  explicit DerivedWithConfig(const Config& config) : config(config) {}

  std::string talk() const override { return config.prefix + "derived" + config.suffix; }

  inline static const auto reg =
      config::RegistrationWithConfig<FooWithConfig, DerivedWithConfig, Config>("derived_with_config");
};

void declare_config(DerivedWithConfig::Config& config) {
  config::name("DerivedWithConfig::Config");
  config::field(config.prefix, "prefix");
  config::field(config.suffix, "suffix");
}

}  // namespace external
