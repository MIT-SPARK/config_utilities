# Overview of functionalities and headers
`config_utilities` is designed as a support library with minimal dependencies (`yaml-cpp` and `C++` standard library).
The library is structured into directories by functionalities and dependencies.
Certain headers contain functionalities to interface with dependencies that are only required by the host project.
These dependencies are pointed out below where present.
The following directories and files exist:

```bash
│                       # All core functionalities are in the unnamed include and depend only on yaml-cpp and C++.
├── config_utilities.h    # Collection of all core headers for easy use.
├── config.h              # To define configs using 'declare_config()'.
├── factory.h             # Enables automatic object creation via 'create()'.
├── globals.h             # Functionality to print all global configs.
├── printing.h            # Defines 'toString()' and 'operator<<' for configs.
├── settings.h            # Enables setting global properties via 'Settings()'
├── traits.h              # Enables 'isConfig()'.
├── validation.h          # Enables 'isValid()' and 'checkVaid()'.
├── virtual_config.h      # Defines 'VirtualConfig' for later factory creation.
│
├── internal            # All files in 'internal' make config_utilities work internally. They have no extra dependencies and need not be included.
│   └── ...
│
├── formatting          # Specify a formatter to parse configs and warnings to text.
│   └── asl.h             # Formatter specialized for human-readable prints to fixed-width consoles (Default).
│
├── logging             # Sepcify an output logger to log warnings and errors to.
│   ├── log_to_glog.h     # Log to glog. Dependes on 'google/logging'.
│   ├── log_to_ros.h      # Log to roslog. Depends on 'ros/console'.
│   └── log_to_stdout.h   # Log to stdout console (Default).
│
├── parsing             # Specify input parsers to get configs or create objects from source data.
│   ├── ros.h             # Tools to create configs and objects from ROS parameter server. Depends on 'ros/nodehandle'
│   └── yaml.h            # Tools to create/save configs and objects from/to yaml nodes or files. Depends on 'yaml-cpp'.
│
└── types               # Support for various types that need special conversions.
    ├── conversions.h     # Support for custom conversions, such as uchars.
    ├── eigen_matrix.h    # Safe and verbose parsing of any Eigen-matrix types.
    ├── enum.h            # Safe and verbose parsing of custom enum types.
    └── path.h            # Conversion for std::filesystem::path and checks for paths and path-strings.
```
