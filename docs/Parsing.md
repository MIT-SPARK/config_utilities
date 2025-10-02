# Parsing configs from data sources

This tutorial explains how to create configs and other objects from source data.

**Contents:**

- [Parse from yaml](#parse-from-yaml)
- [Parse from the command line](#parse-from-the-command-line)
- [Parse via global context](#parse-via-global-context)

## Parse from yaml

To support parsing from yaml nodes and files, the `parsing/yaml.h` header needs to be included. Note that [yaml-cpp](https://github.com/jbeder/yaml-cpp) is already an internal dependency of `config_utilities`, so no additional dependencies are required. `config_utilities` expects yaml files of the format:

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
>
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

## Parse from the command line

It is also possible to use the same interfaces as in the yaml or ROS case but via aggregate YAML read from the command line. To use it, include `parsing/command_line.h`.
`config_utilities` supports parsing the following command line flags:

- `--config-utilities-file SOME_FILE_PATH`: Specify a file to load YAML from.
- `--config-utilities-yaml SOME_ARBITRARY_YAML`: Specify YAML directly from the command line.
- `--config-utilities-var KEY=VALUE`: Specify a new variable for the substitution context.
- `--disable-substitutions/--no-disable-substitutions`: Turn off resolving substitutions

> **✅ Supports**<br>
> Note that the `--config-utilities-file` flag allows for a namespace (i.e., `some/custom/ns`) to apply to the file globally. This is specified as `--config-utilities-file SOME_FILE@some/custom/ns`.

Both command line flags can be specified as many times as needed.
When aggregating the YAML from the command line, the various flags are merged left to right (where conflicting keys from the last specified flag take precedence) and any sequences are appended together.
For those familiar with how the ROS parameter server works, this is the same behavior.
See [here](Compositing.md#controlling-compositing-behavior) for an in-depth discussion of options as to how to control this behavior.
Please also note that the `--config-utilities-yaml` currently accepts multiple space-delimited tokens (because the ROS2 launch file infrastructure does not currently correctly handle escaped substitutions), so

```
some_command --config-utilities-yaml '{my: {cool: config}}' --config-utilities-file some_file.yaml
```

and

```
some_command --config-utilities-yaml {my: {cool: config}} --config-utilities-file some_file.yaml
```

will result in the same behavior (that the resulting parsed YAML will be `{my: {cool: config}}` merged with the contents of `some_file.yaml`).

Parsing directly from the command line takes one of three forms:

```c++
int main(int argc, char** argv) {
  // Instantiate config struct directly.
  MyConfig my_config = config::fromCLI<MyConfig>(argc, argv, "optional/namespace");

  // Factory-based instantiation (where base_args... are arguments to the object constructor)
  const auto object_1 = config::createFromCLI<MyBase>(argc, argv, base_args...);

  // Factory-based instantiation with namespace (where base_args... are arguments to the object constructor)
  const auto object_2 = config::createFromCLI<MyBase>(argc, argv, "optional/namespace", base_args...);
}
```

# Parse via global context

Usually the command line arguments or parsed YAML are not globally available to every part of an executable.
Similar to the ROS1 parameter server (and access to the parameter server by `ros::NodeHandle`), we provide a global `config::internal::Context` object (included via `parsing/context.h`) that handles tracking parsed YAML.
This `config::internal::Context` is not intended to be manipulated directly.
Instead, you should use one of the following methods:

```cpp
int main(int argc, char** argv) {
    // pushes config-utilities specific flags to the end of argv and decrements argc so that it
    // looks like the command was run without any config-utilities specific flags
    const bool remove_config_utils_args = true;
    config::initContext(argc, argv, remove_config_utils_args);

    // adds "{some: {namespace: {a: 5}}}" to the global context
    config::pushToContext(YAML::Load("{a: 5}", "some/namespace"));

    // saves the loaded context
    std::ofstream out("config.yaml");
    out << config::contextToYaml();

    // clears any loaded context
    config::clearContext();
}
```

Please note that the global context is not threadsafe.

This object alllows instantiating configs and objects by the same three forms as the other parsing methods:

```c++
// Instantiate config struct directly.
MyConfig my_config = config::fromContext<MyConfig>("optional/namespace");

// Factory-based instantiation (where base_args... are arguments to the object constructor)
const auto object_1 = config::createFromContext<MyBase>(base_args...);

// Factory-based instantiation with namespace (where base_args... are arguments to the object constructor)
const auto object_2 = config::createFromContext<MyBase>("optional/namespace", base_args...);
```

> **✅ Supports**<br>
> Parsing via the gobal context is the recommended mode, as this supports all functionalities in a simple manner. In addition, [introspection](Introspection.md) works best on the global context.
