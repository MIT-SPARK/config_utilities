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
├── virtual_config.h   # Defines 'VirtualConfig' for later factory creation.
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

# Running Demos

The (non-ros) demos can be run via the `run_demo.py` utility in the scripts directory. If you are building this library via catkin, you can run one of the following:
```
python3 scripts/run_demo.py config
python3 scripts/run_demo.py inheritance
python3 scripts/run_demo.py factory
```
to see the results of one of the corresponding demo files. If you're building via cmake, you can point `run_demo.py` to the build directory with `-b/--build_path`.

The ros demo can be run via
```
roslaunch config_utilities demo_ros.launch
```

# Known Limitatons
- [ ] Minor: Virtual Configs currently do not print defaults. Possible remedies: Make Virtual Configs templated only on creation, or integrate getting default values into C++ struct visitation.

# Sort of nice but niche feature requests
- [ ] Refactor Config checking to get names and types.
- [ ] Make visitors sequential and some extra clean up.
- [ ] Clean up linking, install, and deps.
- [ ] Improve perfrmance (probably reduce yaml copies and check const-ness of yaml nodes)
