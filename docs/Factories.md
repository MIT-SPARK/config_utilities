# Automatic object creation via factories
This tutorial explains how to register derived objects to a factory, to automatically create them from string identifiers, config parsing, or virtual configs.

**Contents:**
- [Automatic object creation](#automatic-object-creation)
- [Creating objects with individual configs](#creating-objects-with-individual-configs)
- [Delayed object creation with virtual configs](#delayed-object-creation-with-virtual-configs)

## Automatic object creation
Objects that derive from a common base can be automatically created using the `config_utilities` factory tools.

> **ℹ️ Note**<br>
> This section covers objects that all have an indentical constructor, whereas objects with specific configs are explained [below](creating-objects-with-individual-configs).

Register object using a static registration struct:
```c++
struct Base {
  virtual void doMagic() = 0;
};

struct DerivedA : public Base {
  virtual void doMagic() override { ... };

 private:
  // Register the class as creatable module for Base with a string identifier using a static registration struct.
  // Signature: Registration<Base, DerivedA, ConstructorArguments...>(string identifier, whether to use a config).
  inline static const auto registration_ = config::Registration<Base, DerivedA>("DerivedA");
};

struct DerivedB : public Base { /* as above */ ... };
```

Then objects can be created using the defined string-identifier:
```c++
std::unique_ptr<Base> object = create<Base>("DerivedA");
```

> **✅ Supports**<br>
> This interface also supports additional constructor arguments. These need to also be declared in the registration. For example:
> ```c++
> struct Base {
>   Base(int i, float f);
> };
>
> struct Derived : public Base {
>   Derived(int i, float f) : Base(i, f) {}
>   inline static const auto registration_ = config::Registration<Base, Derived, int, float>("Derived");
> };
>
> std::unique_ptr<Base> object = create<Base>("Derived", 42, 1.23f);
> ```

> **⚠️ Important**<br>
> Note that different constructor argument template parameters will result in different factories at lookup time. For objects with optional constructor arguments, a registration for each version of the constructor needs to be declared.
Note that references are silently dropped when calling `create` and will result in looking for a different factory constructor than registered. Pay careful attention to the different argument types between the constructor and the registration in the following example!
```c++
struct Bar {
  float f;
};

struct Foo {
  Foo(int i, const Bar& bar) {}
  // Foo is already instantiable as a base class, so it is repeated twice as an argument
  inline static const auto registration_ = config::Registration<Foo, Foo, int, Bar>("Foo");
}
## Creating objects with individual configs

We further provide a version of the factory that allows the declaration of an additional config struct for each derived type. The config is expected to be a `config_utilities` config, the first argument to the constructor, and will be created from the data provided to create the object:

```c++
class Derived : public Base {
 public:
  // Member struct config definition.
  struct Config { ... };

  // Constructore must take the config as first argument.
  DerivedC(const Config& config, const int i) : Base(i), config_(config::checkValid(config)) {}

 private:
  const Config config_;

  // Register the module to the factory with a static registration struct. Signature:
  // RegistrationWithConfig<BaseT, DerivedT, DerivedConfigT, OtherConstructorArguments...>(string identifier).
  inline static const auto registration_ =
      config::RegistrationWithConfig<Base, Derived, Derived::Config, int>("Derived");
};

void declare_config(Derived::Config& config) { ... }

// Create an object from a data source:
std::unique_ptr<Base> = createFromYamlFile<Base>(file_name, 42);
```

Note that the type of object to create is read from the data source. By default, a param with name `type` needs to be set. This can be changed in the `config_utilities` [settings](Varia.md#settings). A sample file could look like:
```yaml
type: "Dervied"
derived_config_param: value
```

> **✅ Supports**<br>
> Additional constructor arguments are supplied separately to the create call. If they are also to be loaded from the data source, you can make them part of the config or a common base config!

## Delayed object creation with virtual configs
[Virtual configs](Types.md#virtual-configs) wrap the above functionality in a config struct. When the virtual config or parent config is setup, the type and content of the virtual config is set and later be used to create `Base` objects:

```c++
struct Config {
  VirtualConfig<Base> base_config;
}

Config config = fromYaml<Config>(data)

// The config uses the 'type' param in the data to set up its type. Once it's setup it can be queried:
std::string type = config.getType();  // E.g. ='Derived' in the above example.

// The virtual config now contains the config and type to create the object:
std::unique_ptr<Base> object = config.create();
```

> **✅ Supports**<br>
> Virtual configs can wrap all the functionality of `createFromDatasource`-style calls in pure C++, if e.g. the datasource is not available in your core project.
