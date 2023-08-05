# Config essentials

## Declaring a struct a config
Any struct or class can be declared to be a `config_utilities`-config. Simply define a function `void declare_config(ConfigT& config)` and the library will work for `ConfigT`:

```c++
struct MyObject { ... };
void declare_config(MyObject& config) { ... }   // Works!
```

> **⚠️ Important:** The declaration of `declare_config` *must* be in the same namespace as the object type being declared.

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

In the body of `declare_config`, we tell `config_utilities` everything it needs to know about a config.

> **⚠️ Important:**<br>
> The declaration of `declare_config` *must* be in the same namespace as the object type being declared.

## Checking for valid configurations

## Printing configs
