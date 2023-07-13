/**
 * TODO(lschmid): Shows how to use configs.
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
#include "config_utilities/types/eigen_matrix.h"     // Enable parsing and printing of Eigen::Matrix types.
#include "config_utilities/validity_checks.h"        // Enable isValid() and checkValid().

namespace demo {

// Declare a base config and object.
struct BaseConfig {
  int i = 1;
  float f = 2.34f;
};

void declare_config(BaseConfig& config) {
  using namespace config;
  name("BaseConfig");
  field(config.i, "i");
  field(config.f, "f");
  checkGT(config.f, 0.f, "f");
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

// Multiple and nested inheritance is supported.
struct BaseBaseConfig {
  std::vector<int> vec = {1, 2, 3};
};

void declare_config(BaseBaseConfig& config) {
  using namespace config;
  name("BaseBaseConfig");
  field(config.vec, "vec");
  checkEQ(config.vec.size(), size_t(3), "vec.size()");
}

struct AnotherBaseConfig : public BaseBaseConfig {
  bool b = true;
};

void declare_config(AnotherBaseConfig& config) {
  using namespace config;
  name("AnotherBaseConfig");
  // Use config::base() to declare that this config inherits from another config.
  base<BaseBaseConfig>(config);
  field(config.b, "b");
  checkEQ(config.b, true, "b");
}

// Declare a derived config and object.
struct DerivedConfig : public BaseConfig, AnotherBaseConfig {
  double d = 5.67;
  std::string s = "Some text";
};

void declare_config(DerivedConfig& config) {
  using namespace config;
  name("DerivedConfig");
  // Multiple inheritance can simply be defined sequentially.
  base<BaseConfig>(config);
  base<AnotherBaseConfig>(config);
  field(config.d, "d");
  field(config.s, "s");
  checkGT(config.d, 0.0, "d");
}

class DerivedObject : public BaseObject {
 public:
  explicit DerivedObject(const DerivedConfig& config) : BaseObject(config), config_(config) {
    config::checkValid(config);
  }

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
  config::Settings().index_subconfig_field_names = true;

  // ===================================== Checking whether a struct is a config =====================================

  // TODO(lschmid): Make this nicer.
  const std::string my_root_path = "/home/lukas/khronos_ws/src/config_utilities/config_utilities/demos/";

  // Create the config like any other.
  auto config = config::fromYamlFile<demo::DerivedConfig>(my_root_path + "demo_inheritance.yaml", "valid_ns");
  bool is_valid = config::isValid(config);
  std::cout << "Config is valid: " << std::boolalpha << is_valid << std::endl;
  std::cout << config << std::endl;

  // Use the derived config to create the derived object.
  demo::DerivedObject object(config);
  object.print();

  // Invalid configs will be warned about as usual
  auto invalid_config = config::fromYamlFile<demo::DerivedConfig>(my_root_path + "demo_inheritance.yaml", "invalid_ns");
  is_valid = config::isValid(invalid_config, true);
  std::cout << "Config is valid: " << std::boolalpha << is_valid << std::endl;
  std::cout << invalid_config << std::endl;

  // TODO(lschmid): This exiting on the BaseConfig check is not the nicest but sort of hard to guarantee that it will be
  // checked for derived...
  demo::DerivedObject invalid_object(invalid_config);
  return 0;
}
