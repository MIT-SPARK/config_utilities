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

/**
 * This demo shows how to use different inheritance patterns with config_utilities.
 */

#include <iostream>
#include <string>
#include <vector>

#include "config_utilities/config.h"                 // Enables declare_config().
#include "config_utilities/formatting/asl.h"         // Simply including this file sets a style to format output.
#include "config_utilities/logging/log_to_stdout.h"  // Simply including this file sets logging to stdout.
#include "config_utilities/parsing/yaml.h"           // Enable fromYamlFile().
#include "config_utilities/printing.h"               // Enable toString()
#include "config_utilities/traits.h"                 // Enables isConfig()
#include "config_utilities/validation.h"             // Enable isValid() and checkValid().

namespace demo {

// Declare a base config and object. Multiple and nested inheritance is supported, including diamond patterns.

struct BaseConfig {
  int i = 1;
  float f = 2.34f;
};

void declare_config(BaseConfig& config) {
  using namespace config;
  name("BaseConfig");
  field(config.i, "i");
  field(config.f, "f");
  check(config.f, CheckMode::GT, 0.f, "f");
}

class BaseObject {
 public:
  explicit BaseObject(const BaseConfig& config) : config_(config) { config::checkValid(config); }

  virtual void print() const {
    std::cout << "I'm a BaseObject and my config has i=" << config_.i << " and f=" << config_.f << std::endl;
  }

 private:
  const BaseConfig config_;
};

struct DifferentBaseConfig : virtual public BaseConfig {
  std::vector<int> vec = {1, 2, 3};
};

void declare_config(DifferentBaseConfig& config) {
  using namespace config;
  name("DifferentBaseConfig");
  // Use config::base() to declare that this config inherits from another config.
  base<BaseConfig>(config);
  field(config.vec, "vec");
  check(config.vec.size(), CheckMode::EQ, 3, "vec.size()");
}

struct AnotherBaseConfig : virtual public BaseConfig {
  bool b = true;
};

void declare_config(AnotherBaseConfig& config) {
  using namespace config;
  name("AnotherBaseConfig");
  base<BaseConfig>(config);
  field(config.b, "b");
  check(config.b, CheckMode::EQ, true, "b");
}

// Declare a derived config and object.
struct DerivedConfig : public DifferentBaseConfig, public AnotherBaseConfig {
  double d = 5.67;
  std::string s = "Some text";
};

void declare_config(DerivedConfig& config) {
  using namespace config;
  name("DerivedConfig");
  // Multiple inheritance can simply be defined sequentially.
  base<DifferentBaseConfig>(config);
  base<AnotherBaseConfig>(config);
  field(config.d, "d");
  field(config.s, "s");
  check(config.d, CheckMode::GT, 0.0, "d");
}

class DerivedObject : public BaseObject {
 public:
  explicit DerivedObject(const DerivedConfig& config) : BaseObject(config), config_(config::checkValid(config)) {}

  void print() const override {
    std::cout << "I'm a DerivedObject, who knows about my bases config i=" << config_.i << " and f=" << config_.f
              << ", and my own config. My Base knows: ";
    BaseObject::print();
  }

 private:
  const DerivedConfig config_;
};

}  // namespace demo

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "invalid usage! expected resource directory as argument" << std::endl;
    return 1;
  }

  const std::string my_root_path = std::string(argv[1]) + "/";

  config::Settings().inline_subconfig_field_names = true;

  // ===================================== Checking whether a struct is a config =====================================

  // Create the config like any other.
  auto config = config::fromYamlFile<demo::DerivedConfig>(my_root_path + "inheritance.yaml", "valid_ns");
  bool is_valid = config::isValid(config);
  std::cout << "Config is valid: " << std::boolalpha << is_valid << std::endl;
  std::cout << config << std::endl;

  // Use the derived config to create the derived object.
  demo::DerivedObject object(config);
  object.print();

  // Invalid configs will be warned about as usual
  auto invalid_config = config::fromYamlFile<demo::DerivedConfig>(my_root_path + "inheritance.yaml", "invalid_ns");
  is_valid = config::isValid(invalid_config, true);
  std::cout << "Config is valid: " << std::boolalpha << is_valid << std::endl;
  std::cout << invalid_config << std::endl;

  try {
    demo::DerivedObject invalid_object(invalid_config);
  } catch (const std::exception& e) {
    std::cout << "Caught exception: " << e.what() << std::endl;
  }
  return 0;
}
