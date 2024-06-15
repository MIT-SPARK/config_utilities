![CMake: Build](https://github.com/MIT-SPARK/config_utilities/actions/workflows/cmake_build.yml/badge.svg) ![ROS Noetic: Build](https://github.com/MIT-SPARK/config_utilities/actions/workflows/catkin_build.yml/badge.svg)


# config_utilities
`config_utilities` is a minimal but powerful C++ library, providing tools to parse, verify, and print C++ config structs and configurable objects.

## Table of contents
- [Credits](#credits)
- [Why `config_utilities`?](#why-config_utilities)
- [Installation](#installation)
- [How to `config_utilities`?](#how-to-config_utilities)
- [Example Projects using `config_utilities`](#example-projects-using-config_utilities)

## Credits
This library was developed by [Lukas Schmid](https://schmluk.github.io/) and [Nathan Hughes](http://mit.edu/sparklab/people.html) at the [MIT-SPARK Lab](http://mit.edu/sparklab), based on functionalities in [ethz-asl/config_utilities](https://github.com/ethz-asl/config_utilities) and [Hydra](https://github.com/MIT-SPARK/Hydra), and is released under a [BSD-3-Clause License](LICENSE)! Additional contributions welcome! This work was supported in part by the Swiss National Science Foundation and Amazon.

## Why `config_utilities`?
Among many other, the key features of `config_utilities` include:
- **Minimal dependencies**: Only C++17 standard library and [yaml-cpp](https://github.com/jbeder/yaml-cpp).
- Declare **any struct a config**, also from external projects:
    ```c++
    namespace external_project {
        void declare_config(ExternalObject& config); // that's all!
    }   // namespace external_project
    ```
- **Minimal** and **clear, human readable interfaces** for config definitions:
    ```c++
    void declare_config(MyConfig& config) {
        using namespace config;
        name("MyConfig");                             // Name for printing.
        field(config.distance, "distance", "m");      // Field definition.
        check(config.distance, GT, 0.0, "distance");  // Ensure 'distance > 0'.
    }
    ```
- **Everything** related to a config is defined in a **single place**.
  ```c++
  void declare_config(MyConfig& config) { /* ALL the information about a config is here */ }
  ```
- Parse any declared config from **various data sources**, **without pulling in dependencies** on data sources into core libraries:
    ```c++
    core.cpp {
        struct MyConfig { ... };
        void declare_config(MyConfig& config) { ... };
    }  // Depends only on C++ and yaml-cpp.

    application.cpp {
        MyConfig config1 = fromYamlFile<MyConfig>(file_path);
        MyConfig config2 = fromRos<MyConfig>(node_handle);
    }  // Pull in dependencies as needed.
    ```
- **Automatically implements** frequently used functionalities, such as **checking** for valid parameters and **formatting** to string:
    ```c++
    MyObject(const MyConfig& config) : config_(checkValid(config)) {
        // Do magic with config_, which has valid parameters only.
        std::cout << config_ << std::endl;
    }
    ```
- Automatic **run-time module creation** based on specified configurations:
    ```cpp
    static auto registration = Registration<Base, MyDerived>("my_derived");

    const std::string module_type = "my_derived";
    std::unique_ptr<Base> module = create(module_type);
    ```
- **Informative, clear**, and **human readable** printing of potentially complicated config structs:
    ```
    =================================== MyConfig ===================================
    distance [m]:                 -0.9876
    A ridiculously long field name that will not be wrapped: Short Value (default)
    A ridiculously long field name that will also not be wrapped:
                                  A really really really ridiculously long string th
                                  at will be wrapped. (default)
    A really really really really really really ridiculously long field name that wi
    ll be wrapped:                A really really really ridiculously long string th
                                  at will also be wrapped. (default)
    vec:                          [5, 4, 3, 2, 1]
    map:                          {b: 2, c: 3, d: 4, e: 5}
    mat:                          [[1, 2, 3],
                                   [4, 5, 6],
                                   [7, 8, 9]]
    my_enum:                      B
    sub_config [SubConfig] (default):
       f:                         0.123 (default)
       s:                         test (default)
       sub_sub_config [SubSubConfig] (default):
          color:                  [255, 127, 0] (default)
          size:                   5 (default)
    ================================================================================
    ```

- **Verbose warnings and errors** if desired for clear and easy development and use:
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

    [ERROR] No module of type 'NotRegistered' registered to the factory for BaseT='d
    emo::Base' and ConstructorArguments={'int'}. Registered are: 'DerivedB', 'Derive
    dA'.

    [ERROR] Cannot create a module of type 'DerivedA': No modules registered to the
    factory for BaseT='demo::Base' and ConstructorArguments={'int', 'float'}. Regist
    er modules using a static config::Registration<BaseT, DerivedT, ConstructorArgum
    ents...> struct.
    ```

## Installation

This package is compatible with `catkin` and `catkin_simple`. Just clone it into your workspace and you should be all set!
```bash
cd ~/catkin_ws/src
git clone git@github.com:MIT-SPARK/config_utilities.git
catkin build config_utilities
```

If you want to build and install without catkin/ROS, that is easy, too! Just clone this repository and build via CMake:
```bash
git clone git@github.com:MIT-SPARK/config_utilities.git
cd config_utilities/config_utilities
mkdir build
cd build
cmake ..
make -j

# optionally install this package
sudo make install
```

## How to `config_utilities`
We provide detailed introductions about everything you need to know about `config_utilities` in the following [tutorials](docs/README.md#tutorials) and some verbose example [demos](docs/README.md#demos) that you can run.

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


# Example Projects using `config_utilities`
Many cool projects are already using this `config_utilities`! (Unfortunately they're mostly currently private, we will list them soon as they become publicly available). Check them out for some more ideas of what `config_utilities` can do!
