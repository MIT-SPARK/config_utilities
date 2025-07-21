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
 * This demo shows how to use dynamic configs, using a ROS dynamic config server and client.
 */

#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "config_utilities/config.h"                         // Enables declare_config().
#include "config_utilities/dynamic_config.h"                 // Enables DynamicConfig and DynamicConfigServer.
#include "config_utilities/logging/log_to_stdout.h"          // Log config_utilities messages.
#include "config_utilities/printing.h"                       // Enable toString()
#include "config_utilities/types/eigen_matrix.h"             // Enable parsing and printing of Eigen::Matrix types.
#include "config_utilities/types/enum.h"                     // Enable parsing and printing of enum types.
#include "config_utilities/virtual_config.h"                 // Enable virtual configs.
#include "config_utilities_ros/ros_dynamic_config_server.h"  // Enable ROS dynamic config server.

namespace demo {

// A sub-config for later use.
struct SubConfig {
  std::string s = "test";
};

// Declaration of the subconfig.
void declare_config(SubConfig& config) {
  using namespace config;
  name("SubConfig");
  field(config.s, "s");
}

// Some virtual configs that can be used to create a config struct that is not known at compile time.
class BaseModule {
 public:
  BaseModule() = default;
  virtual void print() const = 0;
  virtual ~BaseModule() = default;
};

// Module implementations with different configs. The configs are registered with the factory.
class IntModule : public BaseModule {
 public:
  struct Config {
    int i = 0;
  } const config;
  explicit IntModule(const Config& config) : config(config) {}
  void print() const override { std::cout << "IntModule: " << config.i << std::endl; }
  inline static const auto registration_ =
      config::RegistrationWithConfig<BaseModule, IntModule, IntModule::Config>("IntModule");
};

void declare_config(IntModule::Config& config) {
  using namespace config;
  name("IntModule");
  field(config.i, "i");
}

class StringModule : public BaseModule {
 public:
  struct Config {
    std::string s = "hibidi";
  } const config;
  explicit StringModule(const Config& config) : config(config) {}
  void print() const override { std::cout << "StringModule: " << config.s << std::endl; }
  inline static const auto registration_ =
      config::RegistrationWithConfig<BaseModule, StringModule, StringModule::Config>("StringModule");
};

void declare_config(StringModule::Config& config) {
  using namespace config;
  name("StringModule");
  field(config.s, "s");
}

// Defining the config with sub and virtual configs.
struct MyConfig {
  using VirtualModule = config::VirtualConfig<BaseModule, true>;
  int i = 100;
  double distance = 42;
  bool b = true;
  uint8_t uint = 5;
  std::string str = "only";
  std::vector<int> vec = {1, 2, 3};
  std::map<std::string, int> map = {{"a", 1}, {"b", 2}, {"c", 3}};
  Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Identity();
  enum class MyEnum { kA, kB, kC } my_enum = MyEnum::kA;
  SubConfig sub_config;
  VirtualModule virtual_module;

  // For testing.
  std::vector<SubConfig> sub_config_vec = {SubConfig(), SubConfig(), SubConfig()};
  std::map<std::string, SubConfig> sub_config_map = {{"a", SubConfig()}, {"b", SubConfig()}};
  std::vector<VirtualModule> modules;
  std::map<std::string, VirtualModule> module_map = {{"str_mod", VirtualModule(StringModule::Config())},
                                                     {"int_mod", VirtualModule(IntModule::Config())}};
};

// All config properties are specified within declare_config.
void declare_config(MyConfig& config) {
  using namespace config;
  name("MyConfig");
  field(config.i, "i");
  field(config.distance, "distance", "m");
  field(config.b, "b");
  field(config.uint, "uint");
  field(config.str, "str");
  field(config.vec, "vec");
  field(config.map, "map");
  field(config.mat, "mat");
  enum_field(config.my_enum, "my_enum", {"A", "B", "C"});
  field(config.sub_config, "sub_config");
  field(config.virtual_module, "virtual_module");
  field(config.sub_config_vec, "sub_config_vec");
  field(config.sub_config_map, "sub_config_map");
  field(config.modules, "modules");
  field(config.module_map, "module_map");

  check(config.i, CheckMode::GT, 0, "i");
  checkInRange(config.distance, 0.0, 100.0, "distance");
  checkIsOneOf(config.str, {"only", "limited", "strings", "allowed"}, "s");
}

// Declare an object with a dynamic config.
class ObjectWithDynamicConfig {
 public:
  explicit ObjectWithDynamicConfig(const std::string& name, const MyConfig& initial_config = MyConfig())
      : name_(name), config_(name, initial_config) {
    // The above initialization registers the dynamic config with its global identifier nam, where global is within the
    // process and will be resolved to unique name for every node/object. The config is initialized with the initial
    // parameters. The callback is called whenever the config is updated.
    config_.setCallback(std::bind(&ObjectWithDynamicConfig::callback, this));
  }

 private:
  const std::string name_;
  config::DynamicConfig<MyConfig> config_;
  std::vector<std::unique_ptr<BaseModule>> modules_;

  void createAndPrintModules(const MyConfig& config) {
    // Create all modules from the config.
    modules_.clear();
    if (config.virtual_module) {
      modules_.emplace_back(config.virtual_module.create());
    }
    for (const auto& module : config.modules) {
      modules_.emplace_back(module.create());
    }
    for (const auto& [name, module] : config.module_map) {
      modules_.emplace_back(module.create());
    }

    // Have all modules print.
    for (const auto& module : modules_) {
      module->print();
    }
  }

  void callback() {
    // Do something with the new config.
    // For thread safety, dynamic configs always need to be accessed via the get() method., ideally once per callback.
    const auto config = config_.get();
    std::cout << "Received new config for " << name_ << ":\n" << config::toString(config) << std::endl;
    createAndPrintModules(config);
  }
};

class DemoNode : public rclcpp::Node {
 public:
  DemoNode() : Node("demo_node"), obj1_("object_in_the_node"), server_(this) {}

 private:
  // Dynamic config objects can also be declared in the node.
  ObjectWithDynamicConfig obj1_;

  // Anywhere in the node or outside, a dynamic config server can be created which will manage all dynamic configs
  // throughout the node. In the node is preferred, as lifetime will be managed automatically.
  const config::RosDynamicConfigServer server_;
};

}  // namespace demo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // Create some objects with a dynamic config.
  demo::ObjectWithDynamicConfig obj("dynamic_config_object");
  demo::ObjectWithDynamicConfig other_obj("other_object");

  // All standalone dynamic configs will equally be registered.
  config::DynamicConfig<demo::SubConfig> config("standalone_config");

  // Create a ROS node. Dynamic configs as well as the server can also directly live in the node.
  auto node = std::make_shared<demo::DemoNode>();

  // Alternative to the server living in the node, it could also be created here. Note that only one server should be
  // created per node, and embedding the server in the node is preferred as it will automatically manage the lifetime of
  // the server. Thus this is commented out.
  /* config::RosDynamicConfigServer server(node.get()); */

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
