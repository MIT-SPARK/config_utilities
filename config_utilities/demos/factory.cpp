/**
 * TODO: Shows how to use factories.
 */

#include "config_utilities/factory.h"

#include <iostream>
#include <string>

#include "config_utilities/config.h"                 // enables 'create()'
#include "config_utilities/formatting/asl.h"         // Simply including this file sets a style to format output.
#include "config_utilities/logging/log_to_stdout.h"  // Simply including this file sets logging to stdout.

namespace demo {

// Declare a Base and two derived classes.

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

}  // namespace demo

int main(int argc, char** argv) {
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

  return 0;
}
