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
#include "config_utilities/validity_checks.h"        // Enable isValid() and checkValid().

namespace demo {

// A struct that represents we want to be a config.
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
};

// A second struct that will not be declared a config.
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

  // Specify all checks to denote a valid configuration. Checks are specified as param, value, and param name to be
  // displayed. Implemented checks are GT (>), GE (>=), LT (<), LE (<=), EQ (==), NE (!=).
  // TODO(lschmid): Would be nice to not duplicate the name but didn't find a nice way to do this.
  config::checkGT(config.i, 0, "i");

  // Double sided checks can be invoked as in range.
  config::checkInRange(config.distance, 0.0, 100.0, "distance");

  // Any other checks can be implmented using the generic condition check.
  config::checkCondition(config.distance < config.i, "Param 'distance' must be < 'i'.");
  config::checkCondition(!config.s.empty(), "Param 's' may not be empty.");
}

}  // namespace demo

namespace config::internal {
// Specialize this template to get additional information for formatting about custom types.
template <>
std::string getFieldTypeInfo(const int& /* field */) {
  return "int";
}
template <>
std::string getFieldTypeInfo(const bool& /* field */) {
  return "bool";
}

}  // namespace config::internal

int main(int argc, char** argv) {
  std::cout << argv[0] << std::endl;
  // ===================================== Checking whether a struct is a config =====================================

  // Use isConfig<T> to check whether an object has been declared a config.
  std::cout << "MyConfig is a config: " << std::boolalpha << config::isConfig<demo::MyConfig>() << std::endl;
  std::cout << "NotAConfig is a config: " << config::isConfig<demo::NotAConfig>() << std::endl;

  // ====================================== Checking whether a config is valid ======================================

  // Create a valid and an invalid config.
  demo::MyConfig config, invalid_config;
  invalid_config.i = -1;
  invalid_config.distance = 123;
  invalid_config.s.clear();

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

  // Read the config from file.
  const std::string my_file_path =
      "/home/lukas/khronos_ws/src/config_utilities/config_utilities/demos/demo_params.yaml";
  config = config::fromYamlFile<demo::MyConfig>(my_file_path);

  std::cout << "Read values i='" << config.i << "', s='" << config.s << "', distance='" << config.distance
            << "' from file." << std::endl;

  // ======================================== Printing configs to string ========================================

  // Easier automatic printing of all configs with unit and additional information can be done using the toString():
  std::cout << config::toString(config) << std::endl;

  return 0;
}
