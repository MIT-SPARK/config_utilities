/**
 * This demo showcases the use of config_utilities to define configs for custom
 * classes, check these for validity and print them.
 */

#include <iostream>
#include <string>

// // #include "config_utilities.hpp"

// // Define a class that uses a config.
// class MyClass {
//  public:
//   // Any new Config can be defined by inheritnig from config_utility::Config and
//   // templating itself.
//   struct Config : public config_utilities::Config<Config> {
//     int a = 1;
//     double b = 2.34;
//     double b_half = -1.0;
//     std::string c = "this is c";

//     // General fields can be set in the constructor if required.
//     Config() {
//       setConfigName("MyClass-Config");
//     }

//    protected:
//     // To add parameter conditions override 'checkParams()'. The checks can be
//     // performed using the checkParamX tools, which have  identical interfaces
//     // as the 'ConfigChecker'.
//     void checkParams() const override {
//       checkParamGE(a, 0, "a");
//       checkParamEq(c, std::string("this is c"), "c");
//       checkParamCond(static_cast<int>(b) > a, "b is expected > a.");
//     }

//     // Add printing by overriding 'printFields()'. Uniform printing can be
//     // obtained by using the printX tools.
//     void printFields() const override {
//       printField("a", a);
//       printField("b", b);
//       printField("b_half", b_half);
//       printField("c", c);

//       // printField<string>() can be used to create custom, indented messages.
//       printField("An_extremely_unecessarily_and_unreasonably_long_param_name",
//                  "A_similarly_unreasonably_long_param_value.");

//       // Custom text can be printed using printText().
//       printText("And a custom message.");
//     }

//     // Dependent default arguments can be set by overriding this function.
//     void initializeDependentVariableDefaults() override {
//       if (b_half == -1.0) {
//         b_half = b / 2.0;
//       }
//     }
//   };

//   // Use the config for construction. All configs expose the functions
//   // 'bool isValid() const' and 'Config checkValid() const'.
//   explicit MyClass(const Config& config) : config_(config.checkValid()) {}

//   // Use the toString() method for printing.
//   void print() const { std::cout << config_.toString() << std::endl; }

//  private:
//   const Config config_;
// };

int main(int argc, char** argv) {
  // Setup logging.
//   config_utilities::RequiredArguments ra(
//       &argc, &argv, {"--logtostderr", "--colorlogtostderr"});
//   google::InitGoogleLogging(argv[0]);
//   google::ParseCommandLineFlags(&argc, &argv, false);

//   // Create a valid (default) config.
//   MyClass::Config valid_config;
//   MyClass my_class(valid_config);

//   // This should print the config neatly.
//   my_class.print();

//   // Create an invalid config.
//   MyClass::Config invalid_config;
//   invalid_config.a = -1;
//   invalid_config.b = -3;
//   invalid_config.c = "test";

//   // This should raise a warning for every wrong parameter and then exit with a
//   // failed check.
//   MyClass another_my_class(invalid_config);

  return 0;
}
