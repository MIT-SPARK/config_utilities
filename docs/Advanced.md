# Parsing configs from data sources
This tutorial explains how add custom extensions of the functionalities provided in `config_utilities`.

**Contents:**
- [Adding custom types](#adding-custom-types)
- [Adding custom conversions](#adding-custom-conversions)
- [Adding custom checks](#adding-custom-checks)
- [Adding custom loggers](#adding-custom-loggers)
- [Adding custom formatters](#adding-custom-formatters)
- [Adding custom parsers](#adding-custom-parsers)

## Adding custom types
For a type to be supported as a `config_utilities` param, it only needs to be serializable to/from yaml as explained in [their tutorial](https://github.com/jbeder/yaml-cpp/blob/master/docs/Tutorial.md#converting-tofrom-native-data-types).
In short, you can specialize the template struct `YAML::convert` for your type. An example implementation of this is given in `types/eigen_matrix.h`.
```c++
namespace YAML {
template<>
struct convert<MyType> {
  // Implement these two functions.
  static Node encode(const MyType& rhs);
  static bool decode(const Node& node, MyType& rhs);
};
}  // namespace YAML
```

## Adding custom conversions
To implement custom field conversions, you can create a conversion struct. The struct must implement two static conversion functions `toIntermediate` and `fromIntermediate`, where intermediate is a yaml-castable type. Examples of this are given in `types/conversions.h`.

```c++
struct MyConversion {
  // If conversion fails, set 'error' to the failure message.
  static IntermediateType toIntermediate(MyType value, std::string& error);
  static void fromIntermediate(const IntermediateType& intermediate, MyType& value, std::string& error);
};
```

The conversion can now be called on any field definitions by specifying the converter template:
```c++
void declare_config(Config& config) {
   // 'field' will now be read as the intermediate type and will be converted to the underlying config type
  field<MyConversion>(config.field, "field");
}
```


## Adding custom checks
To implement custom checks, inherit from `CheckBase` and implement all virtual functions. Several examples of this are gievn in `checks.h`.
```c++
class MyCheck : public CheckBase {
 public:
  // Constructor takes the arguments of the check call as input to execute the check.
  Check(Arguments... to_check);

  // Return true if the check passed, false if it failed.
  bool valid() const override;

  // The error message for failed checks.
  std::string message() const override;

  // The name of the param (input) for which the check was executed.
  std::string name() const override;

  // Create a copy of this check instance.
  std::unique_ptr<CheckBase> clone() const override ;
};
```

Custom checks can be called using the templated `check` function:
```c++
void declare_config(Config& config) {
   // The arguments are the input to the check constructor.
  check<MyCheck>(config.arguments_to_check);
}
```

## Adding custom loggers
To implement custom loggers, inherit from `Logger` and implement all desired virtual functions. Examples of this are given in `logging/`.

```c++
class MyLogger : public Logger {
 public:
 // Implement this function to do the logging work.
  void logImpl(const Severity severity, const std::string& message) override;

  // Factory registration to allow setting of formatters via Settings::setLogger().
  inline static const auto registration_ = Registration<Logger, MyLogger>("my_logger");

  // Optionally use a static registration struct to set your logger automatically if included.
  inline static const struct Initializer {
    Initializer() { Logger::setLogger(std::make_shared<MyLogger>()); }
  } initializer_;
};
```

> **✅ Supports**<br>
> Loggers can be set using the base-logger's static `setLogger()` function. For example,
> ```c++
> auto logger = std::make_shared<MyLogger>();
> internal::Logger::setLogger(logger);
> { /* do magic */ }
> logger->doSomethingWithCollectedState();
> ```

> **✅ Supports**<br>
> To log to the current `config_utilities` logger, you can use:
> ```c++
> internal::Logger::log(severity, my_message);
> ```

## Adding custom formatters
Formatters work exactly like the loggers above. To implement your custom formatter, inherit from `Formatter` and implement the virtual functions. An example is given in `formatters/asl.h`.

## Adding custom parsers
`config_utilities` uses [yaml-cpp](https://github.com/jbeder/yaml-cpp/tree/master) as internal data representation. To parse configs from other sources of data, convert them to yaml compatible data structures. An example of this is given in `parsing/ros.h`.
