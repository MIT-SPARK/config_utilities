# config_utilities
Utility tools to work with C++ config structs.

# TODO
- [ ] Clean readme
- [ ] Add Docs and how-to for most important features
- [ ] Unit tests

# Structure

`config_utilities` is designed as a support library with minimal dependencies (yaml-cpp and C++ standard library), and additional headers for interfaces with dependencies that are only required by the host project.

Directory of headers to include depending on which functionality you need. All core functionalities are in the unnamed include and depend only on yaml-cpp and C++ std.
```bash
├── config_utilities.h  # Collection of all core headers for easy use.
├── config.h            # To define configs using 'declare_config()'.
├── factory.h           # Enables automatic object creation via 'create()'.
├── printing.h          # Defines 'toString()' and operator<< for configs.
├── settings.h          # Enables setting global properties via 'Settings()'
├── traits.h            # Enables 'isConfig()'.
├── validity_checks.h   # Enables 'isValid()' and 'checkVaid()'.
├── variable_config.h   # Defines 'VariableConfig' for later factory creation.
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
└── types               # Support for other types, such as matrices.
    └── eigen_matrix.h
```



# Known Limitatons
- [ ] Minor: Variable Configs currently do not print defaults. Possible remedies: Make Variable Configs templated only on creation, or integrate getting default values into C++ struct visitation.

# Sort of nice but niche feature requests
- [ ] Refactor Config checking to get names and types.
- [ ] Make visitors sequential and some extra clean up.
- [ ] Clean up linking, install, and deps.
