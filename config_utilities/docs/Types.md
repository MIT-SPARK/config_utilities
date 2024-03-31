# Handling complex configs or types
This tutorial explains how to setup advanced configs structures and types.

**Contents:**
- [Sub-configs](#sub-configs)
- [Inheritance](#inheritance)
- [Virtual configs](#virtual-configs)
- [Type conversions](#type-conversions)
- [Namespaces](#namespaces)

## Sub-configs
Other config structs can be declared a field like any other! Note that the previous config should be declared a config before being used as a sub-config field.
```c++
// Declare the first config.
struct ConfigA { ... };
void declare_config(ConfigA& config) { ... }

// Can be used in other configs as regular fields.
struct ConfigB {
int i;
ConfigA config_a;
};

void declare_config(ConfigB& config) {
name("ConfigB");
field(config.i, "i");
field(config.config_a, "config_a");
}
```
This declaration automatically gets, sets, and checks also the subconfig when `ConfigB` is created, decoded, or checked.

> **ℹ️ Note**<br>
> By default subconfigs are indexed with a namespace that is identical to their field name. For the above example, if ConfigA has a `float` `f`, that would be reflected as:
> ```yaml
> i: value
> config_a:
>   f: value
> ```
This behavior can be overriden using the `field` declaration for subconfigs as follows: <br>
`field(config.config_a, "config_a", false)`. This would yield:
```yaml
i: value
f: value
```

## Inheritance
Configs can inherit from other config structs, e.g. to support derived classes whose bases use configs. Simply declare the base type to be a base of a config:
```c++
// Declare the base config.
struct ConfigA { float f; };
void declare_config(ConfigA& config) { ... }

// Other configs can derive from
struct ConfigB : public ConfigA { int i; };

void declare_config(ConfigB& config) {
name("ConfigB");
base<ConfigA>(config);
field(config.i, "i");
}
```
This declaration automatically gets, sets, and checks also the base config when `ConfigB` is created, decoded, or checked.

> **ℹ️ Note**<br>
> Inheritance uses 'flat' namespaces, e.g. the above example would yield
> ```yaml
> i: value
> f: value
> ```

> **✅ Supports**<br>
> Note that `config_utilities` supports all typical inheritance patterns, such as `single`, `multiple`, `multilevel`, or `hierarchical` inheritance, as well as `diamond patterns`. Simply declare all bases and `config_utilities` will do the rest:
> ```c++
> // Works!
> struct DerivedConfig : public ConfigA, ConfigB { ... };
> void declare_config(DerivedConfig& config) {
> name("DerivedConfig");
> base<ConfigA>(config);
> base<ConfigB>(config);
> }
> ```

## Virtual configs
Virtual configs are structs that store a config needed do create a `BaseObject` (explained in more detail in the chapter [delayed object creation with virtual configs](Factories.md#delayed-object-creation-with-virtual-configs)), where the specific implementation `DerivedObjectX` is not known at compile time. To use virtual configs, include `virtual_config.h`. Example case:

```c++
// Interface or base class to be used.
struct BaseObject {
  struct Config { ... };
  virtual void doStuff() = 0;
};

// Different implementations
struct DerivedA : public BaseObject { struct Config { ... };  ... };
struct DerivedB : public BaseObject { struct Config { ... };  ... };

// Config for an object that holds a BaseObject:
struct Config {
  // This config can hold a DerivedA::Config or DerivedB::Config, which may be completely different objects!
  VirtualConfig<BaseObject> config;
}
```

Virtual configs are treated like any other config! They can e.g. be declared as sub-configs or parsed from data sources and
will autotically loaded, printed, and checked as any other config.

> **ℹ️ Note**<br>
> Note that virtual configs are 'uninitialized' when constructed, and only create the contained config when they or their parent config is [parsed from data](Parsing.md).

By default, uninitialized virtual configs are not considered valid:
```c++
VirtualConfig<Base> config;
isValid(config);  // Will return false with a warning that the config is required but not set.
```
This behavior can be overriden by marking a virtual config as optional:
```c++
VirtualConfig<Base> config;
config.setOptional();
isValid(config);  // Will return true as the config is optional.
```
Similar to uninitialized smart pointers, virtual configs implement `operator bool` to check if they are initialized:
```c++
if (config) {
  // config is initialized and can be used to create the object.
  std::unique_ptr<BaseObject> object = config.create();
} else {
  // Do something with (optional) config that is uninitialized.
}
```

## Type conversions

Certain types may need special conversions or checks. Such `Converter` can be specified as a template argument of the `field` call.

> **ℹ️ Note**<br>
> For just *parsing* of different types, they only need to be yaml-serializable. More details on custom parsing and custom conversions can be found in the [advanced documentation](Advanced.md).

An example use case is parsing of a thread count, where negative values default to the system hardware concurrency.
Such a conversion is implemented in `types/conversions.h` and can be called as follows:
```c++
// Specifying the converter as template argument.
field<ThreadNumConversion>(config.num_threads, "num_threads");
```

Another frequent use case is that of parsing `enum` values. To use this converter, include `types/enum.h`.
The enum converter will parse the enum values to/from human readable string representations and check the values are valid. There are several equivalent ways of declaring the conversion:
```c++
enum class MyEnum { kA, kB, kC};

// Declare the enum conversion globally via a static initializer, so it can be converted everywhere:
auto init = Enum<MyEnum>::Initializer({{MyEnum::kA, "A"}, {MyEnum:kB:, "B"}, {MyEnum::kC, "C"}});

// Afterward the enum conversion can be used in the code:
MyEnum enum_field;
std::string enum_str = Enum<MyEnum>::toString(enum_field);
enum_field = Enum<MyEnum>::fromString(enum_str);

// Config fields can now be declared using the converter:
field<Enum<MyEnum>>(config.enum, "enum");

// Alternatively, the parsing can equivalently be specified directly in the field declaration.
// Note: This can also be used to temporarilly override the global definition:
enum_field<MyEnum>(config.enum, "enum",{{MyEnum::kA, "A"}, {MyEnum:kB:, "B"}, {MyEnum::kC, "C"}});

// For sequential enums, this can also equivalently be declared in short form:
enum_field<MyEnum>(config.enum, "enum",{"A", "B", "C"});
```

## Namespaces
Configs can declare sub-namespaces for their parameters when getting/setting their values. Two equivalent interfaces are provided:
```c++
void declare_config(MyConfig& config){
field(config.a, "a");
enter_namespace("ns1");  // Enters a new namespace 'ns1'
field(config.b, "b");
exit_namespace();  // Exits the last namespace, here 'ns1'
enter_namespace("ns2");
field(config.c, "c");
enter_namespace("ns3");  // Namespaces can also be nested.
field(config.d, "d");
}
```
This will result in:
```yaml
a: value
ns1:
  b: value
ns2:
  c: value
  ns3:
    d: value
```
> **✅ Supports**<br>
> For easier use, `exit_namespace()` followed by `enter_namespace("ns2")` can be replaced by `switch_namespace("n2")`. To exit all open namespaces one can use `clear_namespaces()`.

> **✅ Supports**<br>
> Recall that [subconfig fields](#sub-configs) by default open a namespace with their field name. Any residual namespace left open in a subconfig will be closed when returning to the original config declaration body.

Equivalently, we provide scoped namespace declarations. The below code will produce the same namespaces:
```c++
void declare_config(MyConfig& config){
field(config.a, "a");
{
  NameSpace ns("ns1");  // Scoped namespace definition.
  field(config.b, "b");
}
NameSpace ns("ns2");
field(config.c, "c");
NameSpace more_ns("ns3");
field(config.d, "d");
}
```
