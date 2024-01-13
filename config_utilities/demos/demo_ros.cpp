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
 * This demo shows how to use config_utilities with ROS.
 */

#include <iostream>
#include <string>

#include <glog/logging.h>
#include <ros/ros.h>

#include "config_utilities/config.h"              // Enables declare_config().
#include "config_utilities/formatting/asl.h"      // Simply including this file sets a style to format output.
#include "config_utilities/logging/log_to_ros.h"  // Simply including this file sets logging to roslog.
#include "config_utilities/parsing/ros.h"         // Enable fromRos().
#include "config_utilities/printing.h"            // Enable toString()
#include "config_utilities/traits.h"              // Enables isConfig()
#include "config_utilities/types/eigen_matrix.h"  // Enable parsing and printing of Eigen::Matrix types.
#include "config_utilities/types/enum.h"          // Enable parsing and printing of enum types.
#include "config_utilities/validation.h"          // Enable isValid() and checkValid().

namespace demo {

// A sub-struct for later use.
struct SubConfig {
  float f = 0.123;
  std::string s = "test";
};

// A struct that represents what we want to be a config.
// Requirements for a config struct: is default constructable.
struct MyConfig {
  int i = 100;
  double distance = 42;
  bool b = true;
  std::vector<int> vec = {1, 2, 3};
  std::map<std::string, int> map = {{"a", 1}, {"b", 2}, {"c", 3}};
  Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Identity();
  enum class MyEnum { kA, kB, kC } my_enum = MyEnum::kA;
  SubConfig sub_config;
};

// Defining 'void declare_config(T& config)' function labels a struct as config.
// It **MUST** be declared beforehand if being used in another declare_config
void declare_config(SubConfig&);

// All config properties are specified within declare_config.
void declare_config(MyConfig& config) {
  config::name("MyConfig");
  config::field(config.i, "i");
  config::field(config.distance, "distance", "m");
  config::field(config.b, "b");
  config::field(config.vec, "vec");
  config::field(config.map, "map");
  config::field(config.mat, "mat");
  config::enum_field(config.my_enum, "my_enum", {"A", "B", "C"});
  config::NameSpace ns("sub_ns");
  config::field(config.sub_config, "sub_config");

  config::check(config.i, config::CheckMode::GT, 0, "i");
}

// Declaration of the subconfig.
void declare_config(SubConfig& config) {
  using namespace config;
  name("SubConfig");
  field(config.f, "f");
  field(config.s, "s");
  check(config.f, CheckMode::GT, 0.f, "f");
}

// Declare objects with configs to create from the factory.

class Base {
 public:
  virtual void print() const = 0;
  virtual ~Base() = default;
};

class DerivedA : public Base {
 public:
  struct Config {
    float f = 0.f;
  };

  explicit DerivedA(const Config& config) : config_(config) { config::checkValid(config_); }

  void print() const override { ROS_INFO_STREAM("I'm a DerivedA with config.f='" << config_.f << "'."); }

 private:
  const Config config_;

  // Register the module to the factory with a static registration struct. Signature:
  // RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>(string identifier).
  inline static const auto registration_ = config::RegistrationWithConfig<Base, DerivedA, DerivedA::Config>("DerivedA");
};

void declare_config(DerivedA::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedA");
  config::field(config.f, "f");
  config::check(config.f, config::CheckMode::GE, 0.f, "f");
}

class DerivedB : public Base {
 public:
  struct Config {
    std::string s = "test string";
  };

  explicit DerivedB(const Config& config) : config_(config) {}

  void print() const override { ROS_INFO_STREAM("I'm a DerivedB with config.s='" << config_.s << "'."); }

 private:
  const Config config_;

  inline static const auto registration_ = config::RegistrationWithConfig<Base, DerivedB, DerivedB::Config>("DerivedB");
};

void declare_config(DerivedB::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedB");
  config::field(config.s, "s");
}

}  // namespace demo

int main(int argc, char** argv) {
  ros::init(argc, argv, "demo_ros");
  ros::NodeHandle nh("~");

  // Ther parsing/ros.h header enables getting of configs from ros.
  auto config = config::fromRos<demo::MyConfig>(nh);
  ROS_INFO_STREAM("\n" << config);

  // As well as all factory creation functions.
  nh.setParam("f", 0.123);
  nh.setParam("type", "DerivedA");

  std::unique_ptr<demo::Base> object = config::createFromROS<demo::Base>(nh);
  object->print();

  // Regular config verification and warnings work as usual. Including logging/log_to_ros.h sets the logging to roslog.

  // Set invalid params.
  nh.setParam("type", "A random type");
  nh.setParam("f", -1);
  nh.setParam("i", -1);
  nh.setParam("vec", "Not a vector");

  config = config::fromRos<demo::MyConfig>(nh);
  config::isValid(config, true);
  object = config::createFromROS<demo::Base>(nh);

  ros::shutdown();
  return 0;
}
