# Config essentials

This tutorial explains how to declare a struct to be a config, check for valid configurations, and print configs to string.

**Contents:**
- [Declaring a struct a config](#declaring-a-struct-a-config)
- [Checking for valid configurations](#checking-for-valid-configurations)
- [Printing configs](#printing-configs)

## Declaring a struct a config
Any struct or class can be declared to be a `config_utilities`-config. Simply define a function `void declare_config(ConfigT& config)` and the library will work for `ConfigT`:

```c++
struct MyConfig { ... };
void declare_config(MyConfig& config) { ... }   // Works!
```

> **ℹ️ Note**<br>
> The declaration of `declare_config` *must* be in the same namespace as the object type being declared.

```c++
#include external/other_object.h
namespace external {
    void declare_config(OtherObject& config) { ... }  // Also works!
}   // namespace external
```
```c++
#include external/other_object.h
void declare_config(external::OtherObject& config) { ... }  // Will not work!
```

In the body of `declare_config`, we tell `config_utilities` everything it needs to know about a config. This consists of a `name` used for human readable printing and warnings, `fields` and their names to read and write the config, and `checks` that define whether the value stored in config are valid or not:
```c++
void declare_config(MyConfig& config) {
    config::name("MyConfig");
    // Declares the name to be 'MyConfig'. Does not have to match the truct name but is recmmended.

    config::field(config.int_val, "int_val_name");
    // Declares that 'MyConfig' has a public member 'int_val' that will be referred to as
    // 'int_val_name' for parsing and printing.

    config::field(config.x, "x", "m");
    // Optionally specify a unit as 3rd argument. This will print that x is in nmeters.

    config::check(config.x, config::GT, 0, "x");
    // Declares that the config is only valid if config.x > 0. The second "x" defines the name to
    // be printed if the check fails, and is recommended to be the same as the field name.
}
```

## Checking for valid configurations

A variety of checks are supported by `config_utilities`.
Binary checks such as GT (>), GE (>=), LT (<), LE (<=), EQ (==), NE (!=) can be implemented using `check(field, comparison, reference_value, printing_name)`:
```c++
config::check(config.x, config::GT, 0, "x");
```

For a double sided value check, use `checkInRange(field,, lower, upper, field_name, lower_inclusive, upper_inclusive)`.
E.g. the condition `x is within [0, 100)` becomes:
```c++
  config::checkInRange(config.x, 0.0, 100.0, "x", true, false);
```

Any boolean values can be checked using `config::checkCondition(condition, warning)`:
```c++
config::checkCondition(!config.vector.empty(), "config.vector may not be empty");
```

Custom checks can be implemented and passed as a templated Check Object. This is explained in more details in [ the [advanced features](Advanced.md#adding-custom-checks) tutorial:
```c++
config::check<MyCustomCheck>(custom_arguments);
```

> **ℹ️ Note**<br>
> The above functions are used within `declare_config()` to declare valid values for each field. They *must not* be used outside of it.

To check that a config struct is valid, include `validation.h` and use `isValid(config, print_warnings)`:
```c++
MyConfig cfg;
const bool is_valid = config::isValid(cfg, true);
```
This will print a verbose warning about *all* invalid fields of the config if it fails:
```
=================================== MyConfig ===================================
Warning: Check failed for 'distance': param > 0 (is: '-1').
Warning: Check failed for 'subconfig.f': param within [0, 100) (is: '123').
Warning: Failed to parse param 'vec': Data is not a sequence.
Warning: Failed to parse param 'mat': Incompatible Matrix dimensions: Requested
        3x3 but got 3x4.
Warning: Failed to parse param 'my_enum': Name 'D' is out of bounds for enum wit
        h names ['A', 'B', 'C'].
Warning: Failed to parse param 'uint': Value '-1' underflows storage min of '0'.
================================================================================
```

> **⚠️ Important**<br>
> All failed checks are first formatted by a *formatter* and then printed by a *logger*. If `config_utilities/config_utilities.h` is included these will automatically default to the *asl* formatter and *stdout* logger, resulting in the above print. If no formatter or no logger is included the warning will not show! This holds for all `config_utilities` warnings.

To ensure that a config is valid use `checkValid()`. This will throw and exception or terminate the program if the config is not valid and always print the error.
`checkValid()` return a reference to the config so it can be verified in initializer lists:

```c++
class MyObject {
 public:
  explicit MyObject(const MyConfig& cfg) : config_(config::checkValid(cfg)), object_(config_) {}

private:
  const MyConfig config_;
  ObjectThatNeedsValidConfigs object_;
};
```


## Printing configs

To print configs to string incldue `printing.h`.
This defines `toString()` for objects declared a config:
```c++
MyConfig cfg;
std::cout << config::toString(cfg) << std::endl;
```

This will print the names and all fields declared in the config:
```
=================================== MyConfig ===================================
i:                            123
distance [m]:                 -0.9876
b:                            false
A ridiculously long field name that will not be wrapped: Short Value (default)
A really really really really really really ridiculously long field name that wi
ll be wrapped:                A really really really ridiculously long string th
                              at will also be wrapped. (default)
vec:                          [5, 4, 3, 2, 1]
map:                          {b: 2, c: 3, d: 4, e: 5}
mat:                          [[1, 2, 3],
                               [4, 5, 6],
                               [7, 8, 9]]
my_enum:                      B
my_strange_enum:              X (default)
sub_config [SubConfig]:
   f:                         55555.55
   s:                         test (default)
   sub_sub_config [SubSubConfig]:
      color:                  [255, 255, 255]
      size:                   101010101
================================================================================
```

> **⚠️ Important**<br>
> Printing calls a *formatter* to format the config contents. If `config_utilities/config_utilities.h` is included this will automatically default to the *asl* formatter, resulting in the above print. If no formatter is included `toString()` will return a warning.

It further defines `operator<<` for all objects declared a config, so this is equivalent:
```c++
std::cout << cfg << std::endl;
```

> **⚠️ Important**<br>
> `operator<<` is only defined in namespace `config` or found by argument-dependent lookup (ADL) in the same namespace as the config object is defined. It is thus generally recommended to use `config::toString()` where possible.
