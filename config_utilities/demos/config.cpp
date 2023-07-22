/**
 * TODO(lschmid): Shows how to use configs.
 */

#include "config_utilities/config.h"  // Enables declare_config().

#include <iostream>
#include <string>

#include "config_utilities/formatting/asl.h"         // Simply including this file sets a style to format output.
#include "config_utilities/logging/log_to_stdout.h"  // Simply including this file sets logging to stdout.
#include "config_utilities/parsing/yaml.h"           // Enable fromYamlFile().
#include "config_utilities/printing.h"               // Enable toString()
#include "config_utilities/traits.h"                 // Enables isConfig()
#include "config_utilities/types/eigen_matrix.h"     // Enable parsing and printing of Eigen::Matrix types.
#include "config_utilities/validity_checks.h"        // Enable isValid() and checkValid().

namespace demo {

// A sub-struct for later use.
struct SubSubConfig {
  using Color = Eigen::Matrix<uint8_t, 3, 1>;
  Color color = Color(255, 127, 0);
  size_t size = 5;
};

struct SubConfig {
  float f = 0.123;
  std::string s = "test";
  SubSubConfig sub_sub_config;
};

// A struct that represents what we want to be a config.
// Requirements for a config struct: is default constructable.
struct MyConfig {
  int i = 100;
  std::string s = "test";
  double distance = 42;
  bool b = true;
  std::string test = "Short Value";
  std::string test2 = "A really really really ridiculously long string that will be wrapped.";
  std::string test3 = "A really really really ridiculously long string that will also be wrapped.";
  std::vector<int> vec = {1, 2, 3};
  std::map<std::string, int> map = {{"a", 1}, {"b", 2}, {"c", 3}};
  Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Identity();
  enum class MyEnum { kA, kB, kC } my_enum = MyEnum::kA;
  enum MyStrangeEnum : int { kX = 0, kY = 42, kZ = -7 } my_strange_enum = MyStrangeEnum::kX;
  SubConfig sub_config;
};

// Another struct that will not be declared a config.
struct NotAConfig {};

// Defining 'void declare_config(T& config)' function labels a struct as config. All config properties are specified
// within it.
void declare_config(MyConfig& config) {
  // Specify the name.
  config::name("MyConfig");

  // Specify all fields, optionally specifying a unit for readibility..
  config::field(config.i, "i");
  config::field(config.s, "s");
  config::field(config.distance, "distance", "m");
  config::field(config.b, "b");
  config::field(config.test, "A ridiculously long field name that will not be wrapped");
  config::field(config.test2, "A ridiculously long field name that will also not be wrapped");
  config::field(config.test3,
                "A really really really really really really ridiculously long field name that will be wrapped");
  config::field(config.vec, "vec");
  config::field(config.map, "map");
  config::field(config.mat, "mat");

  // String-based enum conversion and verification is supported. Valid values can be specified as a list of names for
  // sequential enums or as a map for non-sequential enums.
  config::enum_field(config.my_enum, "my_enum", {"A", "B", "C"});
  config::enum_field(
      config.my_strange_enum,
      "my_strange_enum",
      {{MyConfig::MyStrangeEnum::kX, "X"}, {MyConfig::MyStrangeEnum::kY, "Y"}, {MyConfig::MyStrangeEnum::kZ, "Z"}});

  // Sub-namespaces can be used to group fields.
  config::enter_namespace("sub_ns");

  // Any other struct that has been declared a config can be a sub-config, declared as a regular field.
  config::field(config.sub_config, "sub_config");

  // Specify all checks to denote a valid configuration. Checks are specified as param, value, and param name to be
  // displayed. Implemented checks are GT (>), GE (>=), LT (<), LE (<=), EQ (==), NE (!=).
  // TODO(lschmid): Would be nice to not duplicate the name but didn't find a nice way to do this.
  config::check(config.i, config::CheckMode::GT, 0, "i");
  config::check(config.i, config::CheckMode::LT, -2, "i");

  // Double sided checks can be invoked as in range.
  config::checkInRange(config.distance, 0.0, 100.0, "distance");

  // Any other checks can be implmented using the generic condition check.
  config::checkCondition(config.distance < config.i, "Param 'distance' must be < 'i'.");
  config::checkCondition(!config.s.empty(), "Param 's' may not be empty.");
}

// Declaration of the subconfigs.
void declare_config(SubConfig& config) {
  using namespace config;
  name("SubConfig");
  field(config.f, "f");
  field(config.s, "s");
  field(config.sub_sub_config, "sub_sub_config");
  check(config.f, CheckMode::GT, 0, "f");
}

void declare_config(SubSubConfig& config) {
  using namespace config;
  name("SubSubConfig");
  field(config.color, "color");
  field(config.size, "size");
  check(config.size, CheckMode::EQ, size_t(5), "size");
}

}  // namespace demo

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "invalid usage! expected resource directory as argument" << std::endl;
    return 1;
  }

  const std::string my_root_path = std::string(argv[1]) + "/";

  // GLobal settings can be set at runtime to change the behavior and presentation of configs.
  config::Settings().index_subconfig_field_names = true;

  // ===================================== Checking whether a struct is a config =====================================
  std::cout << "\n\n----- Checking whether a struct is a config -----\n\n" << std::endl;

  // Use isConfig<T> to check whether an object has been declared a config.
  std::cout << "MyConfig is a config: " << std::boolalpha << config::isConfig<demo::MyConfig>() << std::endl;
  std::cout << "NotAConfig is a config: " << config::isConfig<demo::NotAConfig>() << std::endl;

  // ====================================== Checking whether a config is valid ======================================
  std::cout << "\n\n----- Checking whether a config is valid \n\n" << std::endl;

  // Create a valid and an invalid config.
  demo::MyConfig config, invalid_config;
  invalid_config.i = -1;
  invalid_config.distance = 123;
  invalid_config.s.clear();
  invalid_config.sub_config.f = -1.f;
  invalid_config.sub_config.sub_sub_config.size = 0;

  // Print whether they are valid. Since we invalidated all fields of 'invalid_config' a comprehensive summary of all
  // issues is printed.
  constexpr bool print_warnings = true;
  const bool config_is_valid = config::isValid(config, print_warnings);
  std::cout << "'config' is valid: " << config_is_valid << std::endl;

  const bool invalid_config_is_valid = config::isValid(invalid_config, print_warnings);
  std::cout << "'invalid_config' is valid: " << invalid_config_is_valid << std::endl;

  // Check valid will enforce that the config is valid, throwing an error and always printing the warnings if not.
  try {
    config::checkValid(invalid_config);
  } catch (std::runtime_error& e) {
    std::cout << "Exception thrown: " << e.what() << std::endl;
  }

  // ======================================== Read the config from file ========================================
  std::cout << "\n\n----- Reading the config from file -----\n\n" << std::endl;

  // Read the config from file.
  config = config::fromYamlFile<demo::MyConfig>(my_root_path + "demo_params.yaml");

  std::cout << "Read values i='" << config.i << "', s='" << config.s << "', distance='" << config.distance
            << "' from file." << std::endl;
  std::cout << "Enum 'config.my_enum' is now B: " << (config.my_enum == demo::MyConfig::MyEnum::kB) << std::endl;

  // Any errors parsing configs will print verbose warnings if desired and use the default values.
  invalid_config = config::fromYamlFile<demo::MyConfig>(my_root_path + "demo_invalid_params.yaml");

  // ======================================== Printing configs to string ========================================
  std::cout << "\n\n----- Printing configs to string -----\n\n" << std::endl;

  // Easier automatic printing of all configs with unit and additional information can be done using the toString():
  const std::string config_as_string = config::toString(config);
  std::cout << config_as_string << std::endl;

  // Inclunding "printing.h" also implements th ostream operator for decared config types. The above is thus equivalent
  // to:
  std::cout << config << std::endl;

  return 0;
}
