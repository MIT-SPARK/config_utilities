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
 * This demo shows how to use the config_utilities library to create a config
 * struct, read it from file, check its validity, and print it to string.
 */

#include <iostream>
#include <stdexcept>
#include <string>

#include "config_utilities/config.h"                 // Enables declare_config().
#include "config_utilities/formatting/asl.h"         // Simply including this file sets a style to format output.
#include "config_utilities/logging/log_to_stdout.h"  // Simply including this file sets logging to stdout.
#include "config_utilities/parsing/yaml.h"           // Enable fromYamlFile().
#include "config_utilities/printing.h"               // Enable toString()
#include "config_utilities/traits.h"                 // Enables isConfig()
#include "config_utilities/types/eigen_matrix.h"     // Enable parsing and printing of Eigen::Matrix types.
#include "config_utilities/types/enum.h"             // Enable parsing and printing of enum types.
#include "config_utilities/validation.h"             // Enable isValid() and checkValid().

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
  // std::map<std::string, int> map;
  Eigen::Matrix<double, 3, 3> mat = Eigen::Matrix<double, 3, 3>::Identity();
  enum class MyEnum { kA, kB, kC } my_enum = MyEnum::kA;
  enum MyStrangeEnum : int { kX = 0, kY = 42, kZ = -7 } my_strange_enum = MyStrangeEnum::kX;
  SubConfig sub_config;
};

// Another struct that will not be declared a config.
struct NotAConfig {};

// Defining 'void declare_config(T& config)' function labels a struct as config.
// It **MUST** be declared beforehand if being used in another declare_config
void declare_config(SubConfig& config);
void declare_config(SubSubConfig& config);

//  All config properties are specified within declare_config.
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
  config::check(config.i, config::GT, 0, "i");
  // config::check(config.i, config::LT, -2, "i");

  // Double sided checks can be invoked as in range.
  const bool lower_inclusive = true;
  const bool upper_inclusive = false;
  config::checkInRange(config.distance, 0.0, 100.0, "distance", lower_inclusive, upper_inclusive);

  // Any other checks can be implmented using the generic condition check.
  config::checkCondition(config.distance < config.i, "param 'distance' must be < 'i'");
  config::checkCondition(!config.s.empty(), "param 's' may not be empty");

  // Any check can be dispatched via templating and forwarded arguments
  config::check<config::internal::CheckRange>(config.distance, 0.0, 100.0, "distance via template");
  config::check<config::internal::Check>(!config.s.empty(), "param 's' may not be empty (via template)");
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

  // std::cout << "Namespace in declare_config " << config::current_namespace();
}

}  // namespace demo


struct SimpleConfig {
  float f = 0.123;
  std::string s = "test";

};

void declare_config(SimpleConfig& config) {
  using namespace config;
  name("SimpleConfig");
  field(config.f, "f");
  field(config.s, "s");
}


#include "config_utilities/internal/visitor.h"
#include <type_traits>
#include <typeindex>
#include <typeinfo>

#include <config_utilities/internal/visitor.h>
#include <config_utilities/traits.h>
#include <config_utilities/validation.h>



//register this to the interface
class ParameterEvenHandler {

  public:
    template<typename ConfigT>
    ParameterEvenHandler(ConfigT* config, const std::string& key) 
    : key_(key), 
      meta_data_(config::internal::Visitor::getValues(*config)), //get meta data from config - we want this for field information etc... TODO: need to do checks
      expected_type_index_(typeid(ConfigT)),
      expected_type_(config::internal::typeName<ConfigT>()) {

      std::cout << "Registereing handler with key " << key << " and type " << expected_type_ << std::endl;

      update_ = [=](const YAML::Node& new_yaml) -> bool {
        if(config) {
          const auto old_yaml = config::internal::Visitor::getValues(*config).data;

          std::cout << "old yaml " << old_yaml << std::endl;

          if (config::internal::isEqual(old_yaml, new_yaml)) {
            std::cout << "Is equal" << std::endl;
            return false;
          }

          //TODO: alert which field has changed!?

          // std::lock_guard<std::mutex> lock(mutex_);
          // TODO(lschmid): We should check if the values are valid before setting them. Ideally field by field...
          ConfigT new_config;
          // std::cout << "new config " << new_config << std::endl;
          config::internal::Visitor::setValues(new_config, new_yaml);
          //update config
          *config = new_config;
          //update hooks!!
          return true;
        }
        else {
          return false;
        }
      };
    }

    template<typename ConfigT>
    bool attemptUpdate(const ConfigT& config) {
      //TODO: check incoming config matches expected type (i.e they are the same!)
      if (expected_type_index_ != typeid(ConfigT)) {
        //TODO: throw exception
        std::cout << "Different types " << expected_type_index_.name() << " is expected vs " << typeid(ConfigT).name() << std::endl;
        return false;
      }

      if (!config::isValid(config, config::Settings().print_warnings)) {
        std::cout << "incoming config with key " << key_ << " is not valid" << std::endl;
        return false;
      }

      //TODO: (recursive lock)?
      const auto new_yaml = config::internal::Visitor::getValues(config).data;
      std::cout << "new yaml " << new_yaml << std::endl;
      return this->update_(new_yaml);
    }

    //TODO: getters
    std::string getKey() const { return key_; }

  private:
    const std::string key_; //top level name
    const config::internal::MetaData meta_data_; //information about the structure of the config
    const std::type_index expected_type_index_;
    const std::string expected_type_;
    //function that does the update
    std::function<bool(const YAML::Node&)> update_;
};



//base class to external interface to monitor for updates to configs
//may be tcp/ros2 etc...
//what to override...?
//I think its easiest to just let the user implement this: e.g in ROS there is no external interface to call
//as the ParameterEventHandler's get triggered by the executor
//TCP or other connections can be added at (derived) construction and the user can start a thread here (or we can have a derived Interface)
//that handles the thread creation. A more defined class interface can be implemeneted at this point. 
class ReconfigureInterface {

  public:
    ReconfigureInterface() = default;
    virtual ~ReconfigureInterface() = default;


    void monitorParam(std::unique_ptr<ParameterEvenHandler> param_handler) {
      //TODO: check that key in param handler is unique
      const auto key = param_handler->getKey();
      parameters_.insert({key, std::move(param_handler)});
    }

  protected:
    //TODO: should not return bool but a more sophsticated 'MetaData' style structure informing the user 
    //what went wrong (or right!!)
    template<typename ConfigT>
    bool set(const ConfigT& config, const std::string& key) {
      if(parameters_.find(key) != parameters_.end()) {
        std::unique_ptr<ParameterEvenHandler>& handler = parameters_.at(key);
        return handler->attemptUpdate(config);
      }
      std::cout << key << " not found";
      return false;
    }

  private:
    std::map<std::string, std::unique_ptr<ParameterEvenHandler>> parameters_; //! All the params being monitored and ones that an be updated

};

// template<typename ConfigT>
class ManualInterface : public ReconfigureInterface {

public:
  // dummy setter for demo
  template<typename ConfigT>
  bool set(const ConfigT& config, const std::string& key) {
    return ReconfigureInterface::set<ConfigT>(config, key);
  }


};

//this is the client?
class DynamicReconfigureServer {

public:
  static DynamicReconfigureServer& instance() {
    static DynamicReconfigureServer instance;
    return instance;
  }

public:
  //interfacce must be derived from ReconfigureInterface
  template<typename Interface>
  static void registerInterface(std::shared_ptr<Interface> interface) {
    //TODO: static assert to enforce type
    DynamicReconfigureServer& server = DynamicReconfigureServer::instance();

    const std::type_index derived_type_index(typeid(Interface));
    std::cout << "Registered interface " << derived_type_index.name() << std::endl;
    //TODO: check that interface is unique? Do we want multiple intferfaces of the same type
    //what about ROS2 nodes? we could have many nodes in the same program (maybe?) and each one would be its own interface of the same type!!
    //if this is the case then we cannot use the Interface type as the key. In this case, we would need to pass an instnace of the interface
    //along with the config in registerConfig, which would totally be okay too!
    
    //Jesse but later: the more I think about this more I think a Singleton Interface is okay, but the user may need do things like add nodes
    //to the interface externally
    server.registered_interface_.insert({derived_type_index, interface});
    

  }

  template<typename Interface, typename ConfigT>
  //key here? or when its registered with declare_config?
  static void registerConfig(ConfigT* config, const std::string& key) {
    //get interface
    DynamicReconfigureServer& server = DynamicReconfigureServer::instance();
    //TODO: check registered
    std::shared_ptr<ReconfigureInterface> interface = server.registered_interface_.at(typeid(Interface));
    assert(interface != nullptr);

    std::cout << "Loaded inteface " << typeid(Interface).name() << std::endl;
    //make new paramter interface to update this config
    auto handler = std::make_unique<ParameterEvenHandler>(config, key);
    //add to interface
    interface->monitorParam(std::move(handler));
  }



private:
  DynamicReconfigureServer() = default;
  //map holding the interfaces and their derived type index so that when a config
  //is registered we can get the right interface
  //this relies on the type index being the same, maybe worth doing some actual casting to check
  std::map<std::type_index, std::shared_ptr<ReconfigureInterface>> registered_interface_;

};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "invalid usage! expected resource directory as argument" << std::endl;
    return 1;
  }

  const std::string my_root_path = std::string(argv[1]) + "/";

  // GLobal settings can be set at runtime to change the behavior and presentation of configs.
  config::Settings().inline_subconfig_field_names = true;
  config::Settings().print_warnings = true;

  SimpleConfig config;
  std::cout << config::toString(config) << std::endl;
  auto interface = std::make_shared<ManualInterface>();
  DynamicReconfigureServer::registerInterface(interface);

  //I am unclear about this key. Does it need to be namespaced according to anything else in the system?
  //Ideally not as its just some key that refers to an INSTANCE of a config and not the Config type (like config::name())
  //some bookeeping could be done here. 
  DynamicReconfigureServer::registerConfig<ManualInterface>(&config, "my_config");

  SimpleConfig new_config;
  interface->set(new_config, "my_config");

  new_config.f = 10.0;
  interface->set(new_config, "my_config");

  std::cout << config::toString(config) << std::endl;

  interface->set(new_config, "non_existant_config");

  //should not work as different config types (between what is registered as "my_config" and the incoming config)
  demo::SubConfig sub_config;
  interface->set(sub_config, "my_config");


  /**
   * @brief Ideas
   * sub config modification via namespacing
   * a config is registered with a name, e.g "my_config" and (using the MetaData) we make namespacing for each of the fields
   * e.g my_config/f, my_config/s
   * 
   * The interface gets a new function that allows sub-updating e.g. Interface::setField(T, namespace)
   * e.g interface->setField(10.1, "my_config/f")
   * 
   * since we have access to my_config and we know how to access the YAML field 'f' associated with that config we can just update the field in the yaml.
   * In this way we dont need to check the type of T against the type of SimpleConfig::f, we just need to check it against the type in the YAML. 
   * Then we can update the new yaml, construt a new SimpleConfig out of it with just the new value for f and update the config as usual.
   * 
   * Of course you can update the whole config by using the "base"/(root?) namespace, which in this example would be "my_config". This is also type checked as we store 
   * the type of SimpleConfig (as is currently implemented)/
   * 
   * This would get around the client side needing to know how to build the WHOLE Config type, you just need to know the namespace and the type of the field (which I guess you 
   * need to do anyway?). The client side wouldn't need to parse around the enture YAML - there might be some complecxity in nested data types however.
   * I think the first version would to work on types that are directly convertable to YAML types (ie. primitives, vectors, maps) or other Config types,
   * since we know how to convert these. 
   *  
   * 
   */

  //update intermediate value
  //TODO: (maybe? this does not work as config.f is not a config)
  //we could make this work if we wanted this type of interface
  // DynamicReconfigureServer::registerConfig<ManualInterface<SimpleConfig>>(&config.f, "my_config_f");




  // config::internal::MetaData md = config::internal::Visitor::getValues(config);
  // std::cout << "Name: " << md.name << "\n";
  // std::cout << "field_name: " << md.field_name << "\n";
  // if (md.map_config_key) std::cout << "map_config_key: " << *md.map_config_key;
  // std::cout << "Data: " << md.data << "\n";
  
  // std::cout << "Field info: \n";
  // for(const auto fi : md.field_infos) {
  //   std::cout << "\t name: " << fi.name << "\n";
  //   std::cout << "\t unit: " << fi.unit << "\n";
  //   std::cout << "\t value: " << fi.value << "\n";
  // }
  // std::cout << std::endl;

  // // ===================================== Checking whether a struct is a config =====================================
  // std::cout << "\n\n----- Checking whether a struct is a config -----\n\n" << std::endl;

  // // Use isConfig<T> to check whether an object has been declared a config.
  // std::cout << "MyConfig is a config: " << std::boolalpha << config::isConfig<demo::MyConfig>() << std::endl;
  // std::cout << "NotAConfig is a config: " << config::isConfig<demo::NotAConfig>() << std::endl;

  // // ====================================== Checking whether a config is valid ======================================
  // std::cout << "\n\n----- Checking whether a config is valid \n\n" << std::endl;

  // // Create a valid and an invalid config.
  // demo::MyConfig config, invalid_config;
  // invalid_config.i = -1;
  // invalid_config.distance = 123;
  // invalid_config.s.clear();
  // invalid_config.sub_config.f = -1.f;
  // invalid_config.sub_config.sub_sub_config.size = 0;

  // // Print whether they are valid. Since we invalidated all fields of 'invalid_config' a comprehensive summary of all
  // // issues is printed.
  // constexpr bool print_warnings = true;
  // const bool config_is_valid = config::isValid(config, print_warnings);
  // std::cout << "'config' is valid: " << config_is_valid << std::endl;

  // const bool invalid_config_is_valid = config::isValid(invalid_config, print_warnings);
  // std::cout << "'invalid_config' is valid: " << invalid_config_is_valid << std::endl;

  // // Check valid will enforce that the config is valid, throwing an error and always printing the warnings if not.
  // try {
  //   config::checkValid(invalid_config);
  // } catch (std::runtime_error& e) {
  //   std::cout << "Exception thrown: " << e.what() << std::endl;
  // }

  // // ======================================== Read the config from file ========================================
  // std::cout << "\n\n----- Reading the config from file -----\n\n" << std::endl;

  // // Read the config from file.
  // config = config::fromYamlFile<demo::MyConfig>(my_root_path + "params.yaml");

  // std::cout << "Read values i='" << config.i << "', s='" << config.s << "', distance='" << config.distance
  //           << "' from file." << std::endl;
  // std::cout << "Enum 'config.my_enum' is now B: " << (config.my_enum == demo::MyConfig::MyEnum::kB) << std::endl;

  // // Any errors parsing configs will print verbose warnings if desired and use the default values.
  // invalid_config = config::fromYamlFile<demo::MyConfig>(my_root_path + "invalid_params.yaml");

  // // ======================================== Printing configs to string ========================================
  // std::cout << "\n\n----- Printing configs to string -----\n\n" << std::endl;

  // // Easier automatic printing of all configs with unit and additional information can be done using the toString():
  // const std::string config_as_string = config::toString(config);
  // std::cout << config_as_string << std::endl;

  // // Inclunding "printing.h" also implements the ostream operator for decared config types. The above is thus equivalent
  // // to:
  // std::cout << config << std::endl;

  return 0;
}
