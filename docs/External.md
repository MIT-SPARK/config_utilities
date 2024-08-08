# Loading External Factories
This tutorial explains how utilize factories in libraries that are not compiled into the current executable.
You should be familiar with how the `config_utilities` factories work to get the most out of this tutorial.
See [here](Factories.md) for an overview.

> **:warning: Warning**<br>
> Loading code from external libraries comes with several caveats.
>   1. Loading an external library executes code by default. Only load external libraries that are from a trusted source!
>   2. Anything instantiated from an external library has to be deallocated before the external library is unloaded.
>   3. Symbol conflicts can occur. See [here](https://www.boost.org/doc/libs/1_64_0/doc/html/boost_dll/tutorial.html#boost_dll.tutorial.symbol_shadowing_problem__linux_)

The features described in this tutorial require including `config_utilities/external_registry.h`.

**Contents:**
- [Loading an external library](#loading-an-external-library)
- [Managed instances](#managed-instances)
- [Debugging](#debugging)

## Loading an external library
To use registered factories from external libraries, you first have to load the libraries themselves.
This is done via `config::loadExternalFactories()`.

Assuming you have a library called `external_logger_plugin` containing the following definition:
```c++
struct ExternalLogger : public config::internal::Loger {
    ... // implementation
    inline static const auto registration = config::Registration<config::internal::Logger, EternalLogger>("my_logger");
};
```

You could instantiate the external logger as follows:
```c++
{ // scope where external library is loaded
    const auto guard = config::loadExternalFactories("external_logger_plugin");
    { // scope to ensure anything instantiated is cleaned up
        const auto external_logger = config::create<config::internal::Logger>("my_logger");
        // do some logging
    } // end scope
} // external library is unloaded after this point

// note that calling create at this point with "my_logger" would fail!
```

> **ℹ️ Note**<br>
> `loadExternalFactories` also supports a vector of libraries as an argument for convenience

Note that in this example (on Linux), we are actually loading the file `libexternal_logger_plugin.so`, which is assumed to be available somewhere on `LD_LIBRARY_PATH`.
You can also load libraries via an absolute path (optionally omitting the `lib` prefix and the `.so` extension).

Either version of `config::loadExternalFactories()` returns scope-based guard(s) that will unload the external libraries as soon as the guard(s) are no longer in scope.
The recommended design for using external libraries is to call `config::loadExternalFactories()` near the beginning of `main()` and then enter a new scope.

## Managed instances

There are often cases where ensuring that resources allocated from an external library are deallocated before the external library is unloaded (e.g., static instances).
In these cases, it may make sense to use the `ManagedInstance` class.

Assuming we have the same external library as before, we can do this instead:
```c++
config::ManagedInstance<config::internal::Logger> external_logger;
{ // scope where external library is loaded
    const auto guard = config::loadExternalFactories("external_logger_plugin");
    external_logger = config::createManaged(config::create<config::internal::Logger>("my_logger"));
    const auto view = external_logger.get();
    view->logInfo("Hello!");
} // external library is unloaded after this point and the external logger no longer works

const auto view = external_logger.get();
// view->logInfo("world!"); // this would segfault

// views can be implicitly converted to a bool to determine whether or not they are valid
if (!view) {
    std::cout << "world!" << std::endl;
}

{ // scope where external library is loaded
    const auto guard = config::loadExternalFactories("external_logger_plugin");
    external_logger = config::createManaged(config::create<config::internal::Logger>("stdout"));
} // external library is unloaded after this point

// the view is still valid in this case as `StdoutLogger` is not in libexternal_logger_plugin.so and doesn't need to be unallocated
const auto new_view = external_logger.get();
view->logInfo("Hello again!");
```

## Debugging
TBD
