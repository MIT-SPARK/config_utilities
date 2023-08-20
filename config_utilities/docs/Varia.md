# Varia

This tutorial explains various additional `config_utilities` functionalities.

**Contents:**
- [Settings](#settings)
- [Globals](#globals)

## Settings
`config_utilities` provides some configuration options that can be set at runtime using the `Settings` struct:
```c++
// Example settings for formatting and printing.
Settings().print_width = 80;

// Example settings for factory creation.
Settings().factory_type_param_name = "type";

// You can set the formatting or logging at runtime.
Settings().setLogger("stdout");
Settings().setFormatter("asl");
```

## Globals
`config_utilities` also provides some preliminary functionalities for global processing. For example, it can keep track of all configs that have been checked for validity using `checkValid()`. This can be used as a proxy for all configs used in a system and can also be disabled in the settings.

```c++
{ /* build a complicated architecture using configs */ }
std::ofstream config_log(log_dest);

// Write the realized configuration of the system, clearing the memory used to store this information.
config_log << Globals().printAllValidConfigs(true);
```
