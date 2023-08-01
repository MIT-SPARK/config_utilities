# config_utilities
`config_utilities` is a minimal but powerful C++ library, providing tools to parse, verify, and print C++ config structs and configurable objects.

## Table of contents
- [Credits](#credits)
- [Why config_utilities](#why-config_utilities)
- [Installation](#installation)
- [How to config_utilities](#how-to-config_utilities)
- [Example Projects using config_utilities](#example-projects-using-config_utilities)
- [Structure](#structure)


## Credits
This library was developed by [Lukas Schmid](https://schmluk.github.io/) and [Nathan Hughes](http://mit.edu/sparklab/people.html) at the [MIT-SPARK Lab](http://mit.edu/sparklab), based on previous functionalities in [ethz-asl/config_utilities](https://github.com/ethz-asl/config_utilities) and [Hydra](https://github.com/MIT-SPARK/Hydra), and is released under a [BSD-3-Clause License](LICENSE)! Additional contributions welcome!

## Why `config_utilities`?
Among many other, the key features of conig_utilities include:
- **Minimal dependencies**: Only C++17 stdandard library and [yaml-cpp](https://github.com/jbeder/yaml-cpp).
- Declare **any struct a config**, also from external projects:
    ```c++
    namespace external_project {
        void declare_config(ExternalObject& config); // that's all!
    }   // namespace external_project
    ```
- Minimal and **clear, human readable interfaces** for config definitions:
    ```c++
    void declare_config(MyConfig& config) {
        using namespace config;
        name("MyConfig");                             // Name for printing.
        field(config.distance, "distance", "m");      // Field definition.
        check(config.distance, GT, 0.0, "distance");  // Ensure 'distance > 0'.
    }
    ```
- **Everything** related to a config is defined in a **single place**.
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
git clone TODO
catkin build config_utilities
```

If you want to build and install without catkin, that should be possible too. Just clone this repository and run:
```bash
cd path/to/this/repo
mkdir build
cd build
cmake ..
make -j

# optionally install this package
sudo make install
```

## How to `config_utilities`
We provide a set of verbose demos covering some of the essential use cases of `config_utilities`.

The (non-ros) demos can be run via the `run_demo.py` utility in the scripts directory. If you are building this library via catkin, you can run one of the following:
```
python3 scripts/run_demo.py config
python3 scripts/run_demo.py inheritance
python3 scripts/run_demo.py factory
```
to see the results of one of the corresponding demo files. If you're building via cmake, you can point `run_demo.py` to the build directory with `-b/--build_path`.

The ros demo can be run via:
```
roslaunch config_utilities demo_ros.launch
```

## Structure

`config_utilities` is designed as a support library with minimal dependencies (yaml-cpp and C++ standard library), and additional headers for interfaces with dependencies that are only required by the host project.

Directory of headers to include depending on which functionality you need. All core functionalities are in the unnamed include and depend only on yaml-cpp and C++ std.
```bash
├── config_utilities.h  # Collection of all core headers for easy use.
├── config.h            # To define configs using 'declare_config()'.
├── factory.h           # Enables automatic object creation via 'create()'.
├── globals.h           # Functionality to print all global configs.
├── printing.h          # Defines 'toString()' and operator<< for configs.
├── settings.h          # Enables setting global properties via 'Settings()'
├── traits.h            # Enables 'isConfig()'.
├── validation.h        # Enables 'isValid()' and 'checkVaid()'.
├── virtual_config.h    # Defines 'VirtualConfig' for later factory creation.
├── internal            # All files in 'internal' are used internally and need not be included.
│   └── ...
├── formatting          # Specify a formatter to parse configs and warnings to text.
│   └── asl.h
├── logging             # Sepcify an output logger to log warnings and errors to.
│   ├── log_to_glog.h
│   ├── log_to_ros.h
│   └── log_to_stdout.h
├── parsing             # Specify input parsers to get configs or create objects from source data.
│   ├── ros.h
│   └── yaml.h
└── types               # Support for various types that need special conversions.
    ├── conversions.h   # Support for custom conversions, such as uchars.
    ├── eigen_matrix.h  # Parsing of any Eigen-matrix types.
    └── enum.h          # Safe and verbose parsing of enum types.
```

## Example Projects using `config_utilities` (TODO)

For additional examples check out these projects using `config_utilities`:
- Hydra
- Khronos

If you are looking for a specific use case that is not in the demos, chances are you can find a good example in the `tests/` directory!

# Old

## Tutorial ideally covered topics:
- Overview of functionalities and headers
- Config essentials:
    - Declaring a struct a config.
    - Checking for valid configurations.
    - Printing configs.
- Parsing configs from data sources:
    - parse from yaml
    - parse from ROS
- Automatic object creation via factories:
    - automatic object creation
    - creating objects with individual configs
- Handling complex configs or types:
    - Sub-configs
    - Inheritance
    - Virtual configs
    - Type conversions
    - Namespaces
- Advanced features:
    - Adding custom type specializations
    - Adding custom checks
    - Adding custom loggers
    - Adding custom formatters
    - Adding custom parsers

# Sort of nice but not high prio feature requests
- [ ] Refactor Config checking to get names and types.
- [ ] Revamp demos.
- [ ] List-based factory creation for processors
- [ ] Make global settings itself a config that can be loaded/saved.
- [ ] Json support?
- [ ] Github CI / Release?
