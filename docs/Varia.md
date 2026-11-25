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

`Settings().factory.strict_virtual_config_types` is `true` by default.
With this setting enabled, a virtual config fails validation if its requested
type could not be loaded by the factory. Set it to `false` to allow optional
virtual configs with a failed lookup to pass validation as unset. The factory
still logs the lookup error.

With `setConfigSettingsFromYAML()` or `setConfigSettingsFromContext()`, use
`factory: {strict_virtual_config_types: false}` to restore the previous behavior.
