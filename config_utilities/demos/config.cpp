/**
 * TODO(lschmid): Shows how to use configs.
 */

#include "config_utilities/config.h"

#include <iostream>
#include <string>

// #include "config_utilities/printing.h"         // enable toString()
#include "config_utilities/formatting/asl.h"         // Specify a style to format output.
#include "config_utilities/logging/log_to_stdout.h"  // Simply including this file sets logging to stdout.
#include "config_utilities/validity_checks.h"        // Enable isValid() and checkValid().

namespace demo {

// A struct that represents we want to be a config.
// Requirements for a config struct: is default constructable.
struct Config {
  int i = 100;
  std::string s = "test";
  double distance = 42;
};

// A second struct that will not be declared a config.
struct NotAConfig {};

// Defining 'void declare_config(T& config)' function labels a struct as config. All config properties are specified
// within it.
void declare_config(Config& config) {
  // Specify the name.
  config::name("Config");

  // Specify all fields, optionally specifying a unit for readibility..
  config::field(config.i, "i");
  config::field(config.s, "s");
  config::field(config.distance, "distance", "m");

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

int main(int argc, char** argv) {
  // Use isConfig<T> to check whether an object has been declared a config.
  std::cout << "Config is a config: " << std::boolalpha << config::isConfig<demo::Config>() << std::endl;
  std::cout << "NotAConfig is a config: " << config::isConfig<demo::NotAConfig>() << std::endl;

  // Create a valid and an invalid config.
  demo::Config config, invalid_config;
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
  config::checkValid(invalid_config);

  return 0;
}
