# Introspection Functionalities

Config Utilities Introspection is primarily a debugging and information tool, that will log detailed information about all get/set and related events happening when loading your configuration. This detailed information will be saved to file and can be rendered to analyze the configuration progress for your app.

## Getting Introspection data

To enable the logging of introspection data, run your app that uses config_utilities with the `--config-utilities-introspection` or `-i` argument.
By default, introspection output will be written to `./config_introspection_results` and stored in a file called `data.json`.
Optionally, a different target target directory can be specified using `-i /my/output/path`.

This works automatically when calling `config::initContext(argc, argv)` in your main file, which is requried to parse the command line arguments.
Alternatively, the introspection can be enabled by setting the `config::Settings().introspection.output` field to a directory before running any other parsing. In this case, the output data will be written to the specified output drectory.

> **✅ Supports**<br>
> When `-i` is set anywhere in the input arguments, all steps of loading and changing of configs will automatically be covered in the introspection. This is the recommended mode of usage.

> **⚠️ Important**<br>
> Currently introspection only captures events via the Global Context (see [Parse via global context](Parsing.md#parse-via-global-context)). This is the preferred method to configure your app. Introspection results for other forms of parsing may be supported in the future.

## Visualize the results

To interactively visualize the results, we provide the `config-utilities-viewer` tool.
After running your app with `-i` option, you can run `config-utilities-viewer` from the same terminal to inspect the results.
If running in the same directory, it will automatically look for the default introspection output. Otherwise, use the `--data /my/output/path/data.json` arg to read the correct introspection output.
Your browser should automatically open with the viewer, otherwise navigate to to host:port (default is `https://localhost:5000`) to see the output.
