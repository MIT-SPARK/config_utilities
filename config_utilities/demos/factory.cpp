/**
 * TODO: Shows how to use factories.
 */

#include "config_utilities/factory.h"  // enables 'create()'

#include <iostream>
#include <string>

#include "config_utilities/config.h"                 // Required for config declaration
#include "config_utilities/formatting/asl.h"         // Simply including this file sets a style to format output.
#include "config_utilities/logging/log_to_stdout.h"  // Simply including this file sets logging to stdout.
#include "config_utilities/parsing/yaml.h"           // enables 'createFromYaml()' and 'createFromYamlFile()'
#include "config_utilities/printing.h"               // enables 'toString()'
#include "config_utilities/validity_checks.h"        // enables 'checkValid()'
#include "config_utilities/virtual_config.h"         // enables 'VirtualConfig'

namespace demo {

// Declare a Base and two derived classes that don't use configs.

class Base {
 public:
  explicit Base(int i) : i_(i) {}
  virtual ~Base() = default;
  virtual void print() const { std::cout << "I'm a Base with i='" << i_ << "'." << std::endl; }

 protected:
  const int i_;
};

class DerivedA : public Base {
 public:
  explicit DerivedA(int i) : Base(i) {}
  void print() const override { std::cout << "I'm a DerivedA with i='" << i_ << "'." << std::endl; }

 private:
  // Register the class as creatable module for Base with a string identifier using a static registration struct.
  // Signature: Registration<Base, DerivedA, ConstructorArguments...>(string identifier, whether to use a config).
  inline static const auto registration_ = config::Registration<Base, DerivedA, int>("DerivedA");
};

class DerivedB : public Base {
 public:
  explicit DerivedB(int i) : Base(i) {}
  void print() const override { std::cout << "I'm a DerivedB with i='" << i_ << "'." << std::endl; }

 private:
  inline static const auto registration_ = config::Registration<Base, DerivedB, int>("DerivedB");
};

class NotOfBase {};

// Declare two derived classes that use configs.

class DerivedC : public Base {
 public:
  // Member struct config definition.
  struct Config {
    float f = 0.f;
  };

  // Constructore must take the config as first argument.
  DerivedC(const Config& config, const int& i) : Base(i), config_(config::checkValid(config)) {
    // Make sure the config is valid, otherwise throw an exception.
  }

  void print() const override {
    std::cout << "I'm a DerivedC with i='" << i_ << "' and config.f='" << config_.f << "'." << std::endl;
  }

 private:
  const Config config_;

  // Register the module to the factory with a static registration struct. Signature:
  // RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, ConstructorArguments...>(string identifier).
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base, DerivedC, DerivedC::Config, int>("DerivedC");
};

void declare_config(DerivedC::Config& config) {
  // Declare the config using the config utilities.
  config::name("DerivedC");
  config::field(config.f, "f");
  config::checkGE(config.f, 0.f, "f");
}

// Configs can be defined anywhere, e.g. also structs from different libraries as long as a 'void
// declare_config(ConfigT& config)' exists.
struct ExternalConfig {
  std::string s = "I'm a completely different field from float f.";
};

void declare_config(ExternalConfig& config) {
  // Declare the config using the config utilities.
  using namespace config;
  name("ExternalConfig for DerivedD");
  field(config.s, "s");
}

class DerivedD : public Base {
 public:
  DerivedD(const ExternalConfig& config, const int& i) : Base(i), config_(config) {}

  void print() const override {
    std::cout << "I'm a DerivedD with i='" << i_ << "' and config.s='" << config_.s << "'." << std::endl;
  }

 private:
  const ExternalConfig config_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<Base, DerivedD, ExternalConfig, int>("DerivedD");
};

// Define an object that uses virtual configs to specify its members.
class ObjectWithDerivedMembers {
 public:
  struct Config {
    double d = 0.123;
    config::VirtualConfig<Base> base_config;
  };

  explicit ObjectWithDerivedMembers(const Config& config, int base_i) : config_(config) {
    config::checkValid(config_);
    base_ = config_.base_config.create(base_i);
  }

  void print() const {
    std::cout << "I'm an ObjectWithDerivedMembers with d='" << config_.d << "' and base object: ";
    base_->print();
  }

 private:
  const Config config_;
  std::unique_ptr<Base> base_;
};

void declare_config(ObjectWithDerivedMembers::Config& config) {
  using namespace config;
  name("ObjectWithDerivedMembers");
  field(config.d, "d");
  // Variable configs are declared like regular sub-configs and work exactly like them.
  subconfig(config.base_config, "base_config");
}

}  // namespace demo

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "invalid usage! expected resource directory as argument" << std::endl;
    return 1;
  }

  const std::string my_root_path = std::string(argv[1]) + "/";

  // --------------- Creating strig-identified objects ----------------

  // Create an object of type Base using the factory.
  const std::string type = "DerivedA";
  const int value = 42;
  std::unique_ptr<demo::Base> object = config::create<demo::Base>(type, value);

  // This should say object is in fact of type DerivedA.
  object->print();

  // Requesting a type that is not registered will throw a verbose exception. Needs to be compiled with symbols,
  // e.g. -DCMAKE_BUILD_TYPE=RelWithDebInfo to be human readable.
  object = config::create<demo::Base>("NotRegistered", value);

  // Requesting and invalid type will throw a verbose exception.
  auto other_object = config::create<demo::NotOfBase>("DerivedA", value);

  // Note that changing th constructor arguments changes the signature. If multiple constructors are available, the
  // all versions need to be registered with their own registration struct.
  object = config::create<demo::Base>("DerivedA", value, 1.f);

  // --------------- Creating objects from file ----------------

  // Optionally specify the name of the type-identifying param. Default is 'type'.
  config::Settings().factory_type_param_name = "type";

  // Create an object of type and with config as specified in a file.
  object = config::createFromYamlFile<demo::Base>(my_root_path + "demo_factory.yaml", 123);
  object->print();

  // By changing the type param, a different object can be created.
  YAML::Node file_data = YAML::LoadFile(my_root_path + "demo_factory.yaml");
  file_data["type"] = "DerivedD";
  object = config::createFromYaml<demo::Base>(file_data, 123);
  object->print();

  // Objects can also be created from separate namespaces.
  object = config::createFromYamlFileWithNamespace<demo::Base>(my_root_path + "demo_factory.yaml", "special_ns", 123);
  object->print();

  // If the type param is not specified, a warning will be printed.
  object =
      config::createFromYamlFileWithNamespace<demo::Base>(my_root_path + "demo_factory.yaml", "nonexistent_ns", 123);

  // --------------- Storing Configs to Variable Objects ----------------

  // Variable configs can be used to store configs of any derived type and create that object later.
  config::VirtualConfig<demo::Base> config;

  // An unitnitilized config is not valid.
  std::cout << "Config is set: " << std::boolalpha << config.isSet() << std::endl;
  bool is_valid = config::isValid(config, true);
  std::cout << "Config is valid: " << is_valid << std::endl;
  std::cout << "Config has type: " << config.getType() << std::endl;
  std::cout << "Config content:\n" << config::toString(config) << std::endl;

  // Variable configs are created as any other.
  config = config::fromYamlFile<config::VirtualConfig<demo::Base>>(my_root_path + "demo_factory.yaml");

  std::cout << "Config is set: " << config.isSet() << std::endl;
  is_valid = config::isValid(config, true);
  std::cout << "Config is valid: " << is_valid << std::endl;
  std::cout << "Config has type: " << config.getType() << std::endl;
  std::cout << "Config content:\n" << config::toString(config) << std::endl;

  // Variable configs can be treated like any other as subconfigs.
  auto object_config = config::fromYamlFile<demo::ObjectWithDerivedMembers::Config>(my_root_path + "demo_factory.yaml");

  std::cout << "Object config content:\n" << config::toString(object_config) << std::endl;

  // Most importantly, they can delay the creation of the configured object, as is done in the constructor of
  // 'ObjectWithDerivedMembers'.

  demo::ObjectWithDerivedMembers object_with_members(object_config, 987);
  object_with_members.print();

  return 0;
}
