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
  inline static const auto registration_ = config::Registration<Base, DerivedA, int>("DerivedA");
};
```

## Creating objects with individual configs


## Delayed object creation with virtual configs
