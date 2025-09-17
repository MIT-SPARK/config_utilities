# Introspection Functionalities

- Introspection writes details ofhow all configs were parsed to file for debugging and proves nice viz.
- Run conf utils with `--config-utilities-introspection` or `-i` argument to enable introspection. Optionally specify a target directory `-i /my/output/path`, defaults to `config_introspection_results`. This works automatically when calling `initContext(argc, argv)`.
  - Alternatively, the introspection can be enable by setting the `introspection.output` field to a valid value before running any other parsing.

> **✅ Supports**<br>

> **⚠️ Important**<br>

- Currently only captures set and get events from parsing via Global Context [see context](todo).
