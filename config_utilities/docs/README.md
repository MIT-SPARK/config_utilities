# `config_utilities` Tutorials
We provide detailed introductions about everything you need to know about `config_utilities` in the following [tutorials](#tutorials) and some verbose example [demos](#demos) that you can run.

## Tutorials
The following tutorials will guide you through functionalities of `config_utilities`, from beginner to expert:

1. [**Overview of functionalities and headers**](Headers.md)

2. [**Config essentials**](Config.md)
    - [Declaring a struct a config](Config.md#declaring-a-struct-a-config)
    - [Checking for valid configurations](Config.md#checking-for-valid-configurations)
    - [Printing configs](Config.md#printing-configs)

3. [**Parsing configs from data sources**](Parsing.md)
    - [Parse from yaml](Parsing.md#parse-from-yaml)
    - [Parse from ROS](Parsing.md#parse-from-ros)

4. [**Handling complex configs or types**](Types.md)
    - [Sub-configs](Types.md#sub-configs)
    - [Inheritance](Types.md#inheritance)
    - [Virtual configs](Types.md#virtual-configs)
    - [Type conversions](Types.md#type-conversions)
    - [Namespaces](Types.md#namespaces)

5. [**Automatic object creation via factories**](Factories.md)
    - [Automatic object creation](Factories.md#automatic-object-creation)
    - [Creating objects with individual configs](Factories.md#creating-objects-with-individual-configs)

6. [**Advanced features**](Advanced.md)
    - [Adding custom type specializations](Advanced.md#adding-custom-type-specializations)
    - [Adding custom checks](Advanced.md#adding-custom-checks)
    - [Adding custom loggers](Advanced.md#adding-custom-loggers)
    - [Adding custom formatters](Advanced.md#adding-custom-formatters)
    - [Adding custom parsers](Advanced.md#adding-custom-parsers)


## Demos
The (non-ros) demos can be run via the `run_demo.py` utility in the scripts directory. If you are building this library via catkin, you can run one of the following to see the results of one of the corresponding demo files:
```
python3 scripts/run_demo.py config
python3 scripts/run_demo.py inheritance
python3 scripts/run_demo.py factory
```

> **ℹ️ Note**<br>
> If you're building via cmake, you can point `run_demo.py` to the build directory with `-b/--build_path`.

The ros demo can be run via:
```
roslaunch config_utilities demo_ros.launch
```

If you are looking for a specific use case that is not in the tutorials or demos, chances are you can find a good example in the `tests/` directory!
