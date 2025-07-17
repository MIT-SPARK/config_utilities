# Dynamic Configs

This tutorial demonstrates how to register configs as dynamic, i.e., they can receive get/set requests at runtime to update their values, and how to use the dynamic config server to access these configs.


**Contents:**
- [Declaring a dynamic config](#declaring-a-dynamic-config)
- [Dynamic Config Callbacks](#dynamic-config-callbacks)
- [Setting Dynamic Configs](#setting-dynamic-configs)
- [Custom Dynamic Config Servers](#custom-dynamic-config-servers)


## Declaring a dynamic config
Any `config_utilities` config can be declared as a dynamic config. For this simply use the `DynamicConfig` struct.
Note that all dynamic configs must have a unique key that used to get/set their values:

```c++
#include <config_utilities/dynamic_config.h>

// Define your configs as usual in some header:
struct MyConfig { ... };
void declare_config(MyConfig& config) { ... }

// Instantiate a dynamic config:
config::DynamicConfig<MyConfig> dynamic_config("my_config"); // key: my_config
```

Dynamic configs are thread-safe, you can get and set their values as follows:

```c++
// Getting single values:
if (dynamic_config.get().some_param >= some_value) {
    doMagic();
}
```

> **ℹ️ Note**<br>
> Dynamic config `get()` returns a copy for thread safety, to get multiple values, this is more efficient:

```c++
// Getting multiple values:
const auto current_config = dynamic_config.get();
float z = current_config.x * current_config.y;
```

You can also set dynamic configs using the `set()` function. Note that this takes an entire config as input. To only update specific values, get the current config first:

```c++
// Set the dynamic config to a config:
MyConfig new_values;
dynamic_config.set(new_values); // Works! Sets all values.

// To update single params, get the config first:
auto values = dynamic_config.get();
values.x = new_x;
dynamic_config.set(values);  // Works! Will only update MyConfig.x.

// Recall that config.get() returns a copy:
dynamic_config.get().x = new_x; // Won't work! Only modifies the copy.
```

## Dynamic Config Callbacks

All dynamic configs allow registering callback functions that are triggered whenever the dynamic config is updated:

```c++
config::DynamicConfig<MyConfig> dynamic_config("my_config"); 

// Register a callback:
dynamic_config.setCallback([&dynamic_config](){
  std::cout << "Got new config values: " << config::toString(dynamic_config) << std::endl;
});
```

## Setting Dynamic Configs
We provide a base interface to set dynamic configs via the `DynamicConfigServer`. The server can get configs by key and uses `YAML` as an interface to interact with the config:

```c++
#include <config_utilities/dynamic_config.h>

config::DynamicConfig<MyConfig> dynamic_config("my_config"); // key: my_config

// Servers can be created anywhere in the process to interface with all existing dynamic configs.
config::DynamicConfigServer server;

// Get the values of a dynamic config:
YAML::Node values = server.get("my_config");  

// Set the values of a dynamic config:
YAML::Node new_values = ...;
server.set("my_config", new_values);  // Works! 

// Note that the new values can also contain only a subset of params:
server.set("my_config", YAML::Load("x: 123"));  // Works! Only sets the x param.
```

Similarly to the dynamic configs, also the server allows registering hooks to keep track of which configs are registered and updated:

```c++
DynamicConfigServer::Hooks hooks;
hooks.onRegister = { ... };
hooks.onDeregister = { ... };
hooks.onUpdate = { ... };

server.setHooks(hooks); // All done!
```

To see further functionalities and use cases see the demo and source code.

## Custom Dynamic Config Servers
Custom servers or client can easily be implemented by building on top of the provided `DynamicConfigServer`. 
An example of this is given in the `RosDynamicConfigServer` in `config_utilities_ros`, which advertizes all config get/set interfaces via ROS2 topics.
This can be used to, for example, modify the C++ configs using a python GUI, as demonstrated in our `demo_dynamic_config`. Give it a try:

```bash
 # Required for the GUI:
pip install flask webbrowser  

# Run the demo:
ros2 launch config_utilities_ros demo_ros_dynamic_config.yaml
```
