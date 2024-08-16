# Varia

This tutorial explains various additional `config_utilities` functionalities.

**Contents:**
- [Settings](#settings)

## Settings
`config_utilities` provides some configuration options that can be set at runtime using the `Settings` struct:
```c++
#include <config_utilities/settings.h>
// Example settings for formatting and printing.
Settings().printing.width = 80;

// Example settings for factory creation.
Settings().factory.type_param_name = "type";

// You can set the formatting or logging at runtime.
Settings().setLogger("stdout");
Settings().setFormatter("asl");
```
