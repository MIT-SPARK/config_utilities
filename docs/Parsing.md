# Parsing configs from data sources
This tutorial explains how to create configs and other objects from source data.

**Contents:**
- [Parse from yaml](#parse-from-yaml)
- [Parse from ROS](#parse-from-ros)

## Parse from yaml

To support parsing from yaml nodes and files, the `parsing/yaml.h` header needs to be included. Note that [yaml-cpp](https://github.com/jbeder/yaml-cpp) is already an internal dependecy of `config_utilities`, so no additional dependencies are required. `config_utilities` expects yaml files of the format:
```yaml
namespace:
  field_name: field_value
```

Configs can be created using the `fromYaml()` function:
```c++
// Create a config from a yaml-node:
YAML::Node data = ...;
MyConfig config = fromYaml<MyConfig>(data);

// This call also supports additional namespaces:
const std::string ns = "a/b/c"
MyConfig config = fromYaml<MyConfig>(data, ns);

// This is equivalent to calling (except with additional safety checking):
MyConfig config = fromYaml<MyConfig>(data["a"]["b"]["c"]);
```

Identical interfaces are available to read configs from yaml files:
```c++
// Reading the main namespace.
const std::string file_path = "/path/to/my_config.yaml"
MyConfig config = fromYamlFile<MyConfig>(file_path);

// Reading a specific namespace:
MyConfig config = fromYamlFile<MyConfig>(file_path, "a/b/c");
```

> **✅ Supports**<br>
> Note that the regular `config_utilities` creation interface also supports getting vectors of multiple configs or [virtual configs](Factories.md#delayed-object-creation-with-virtual-configs):
> ```c++
> // Works! Expects the yaml data to contain a list of configs, e.g. [{config1 params}, {config2 params}, ...].
> std::vector<MyConfig> configs = fromYamlFile<std::vector<MyConfig>>(file_path, ns);
>
> // Works, too!
> VirtualConfig<MyBase> virtual_config = fromYaml<VirtualConfig<MyBase>>(file_path, ns);
> ```

For yaml-parsing, `config_utilities` also supports serializing config structs back to yaml data:
```c++
MyConfig config;
YAML::Node node = toYaml(config);
// node will look something like {field: value, ...}

toYamlFile(config, file_name);
```

Lastly, to use [factory creation with configs](Factories.md#creating-objects-with-individual-configs) from yaml data, use:
```c++
std::unique_ptr<MyBase> object = createFromYaml<MyBase>(node);
std::unique_ptr<MyBase> object = createFromYamlWithNamespace<MyBase>(node, ns);
std::unique_ptr<MyBase> object = createFromYamlFile<MyBase>(file_name);
std::unique_ptr<MyBase> object = createFromYamlFileWithNamespace<MyBase>(file_name, ns);
```

## Parse from ROS

Parsing parameters from ros supports the same interfaces as in the yaml case, but reading from the ros parameter server. To use it, simply include `parsing/ros.h`. This pulls in a dependency on `ros/node_handle.h`. `config_utilities` expects params to follow the format `namespace/field_name: field_value`:

```c++
ros::NodeHandle nh("~");

MyConfig config = fromRos<MyConfig>(nh);

// Identically, these also work:
std::vector<MyConfig> configs = fromRos<std::vector<MyConfig>>(nh);
VirtualConfig<MyBase> virtual_config = fromRos<VirtualConfig<MyBase>>(nh);

// A namespaced version is also available. These two statements are identical:
MyConfig config = fromRos<MyConfig>(nh, "a/b/c");
MyConfig config = fromRos<MyConfig>(ros::NodeHandle(nh, "a/b/c"));
```

To use [factory creation with configs](Factories.md#creating-objects-with-individual-configs) from ros data, use:

```c++
std::unique_ptr<MyBase> object = createFromRosMyBase>(nh);
std::unique_ptr<MyBase> object = createFromRosWithNamespace<MyBase>(nh, ns);
```
