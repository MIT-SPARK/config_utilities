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

Assuming you have the following definitions:
```c++
/******** compiled in main executable  **********************************************************************/

namespace talker {

struct Talker {
    virtual std::string talk() const = 0;
};

}  // namespace talker

/******** compiled in a separate library (`libexternal_talker_plugin.so`) *************************************/

namespace external {

struct ExternalTalker : public Talker {
    std::string talk() const { return "Hello"; }
    inline static const auto registration = config::Registration<talker::Talker, ExternalTalker>("my_talker");
};

}  // namespace external
```

You could instantiate `ExternalTalker` from the external library as follows:
```c++
{ // scope where external library is loaded
    const auto guard = config::loadExternalFactories("external_talker_plugin");
    { // scope to ensure anything instantiated is cleaned up
        const auto talker = config::create<talker::Talker>("my_talker");
        std::cout << talker.talk() << std::endl;  // should get 'Hello'
    } // end scope
} // external library is unloaded after this point

// note that calling create at this point with "my_talker" would fail!
```

> **ℹ️ Note**<br>
> `loadExternalFactories` also supports a vector of libraries as an argument for convenience

Note that in this example (on Linux), we are actually loading the file `libexternal_talker_plugin.so`, which is assumed to be available somewhere on `LD_LIBRARY_PATH`.
You can also load libraries via an absolute path (optionally omitting the `lib` prefix and the `.so` extension).

Either version of `config::loadExternalFactories()` returns a scope-based guard that will unload the external libraries as soon as the guard is no longer in scope.
The recommended design for using external libraries is to call `config::loadExternalFactories()` near the beginning of `main()` and then enter a new scope.

## Managed instances

There are often cases where ensuring that resources allocated from an external library are deallocated before the external library is unloaded (e.g., static instances).
In these cases, it may make sense to use the `ManagedInstance` class.

Assuming we have the same external library as before, we can do this instead:
```c++
config::ManagedInstance<talker::Talker> talker;
{ // scope where external library is loaded
    const auto guard = config::loadExternalFactories("external_talker_plugin");
    talker = config::createManaged(config::create<talker::Talker>("my_talker"));
    const auto view = talker.get();
    std::cout << view->talk() << std::end;  // should get 'Hello'
} // external library is unloaded after this point and the managed instance is no longer valid

const auto view = talker.get();
// view->talk(); // this would segfault

// views can be implicitly converted to a bool to determine whether or not they are valid
if (!view) {
    std::cout << "world!" << std::endl;
}
```

> **:warning: Warning**<br>
> All `ManagedInstance` instantiations will be invalidated as soon as any library is unloaded.
> There is no safe way to determine whether or not the underlying types still have the necessary underlying libraries available.

Note that `ManagedInstance` may be instantiated by factories that are already compiled into the executable.

## Debugging

Tracking down issues with code loaded from external libraries can be hard.
You may find it helpful to turn on allocation logging by doing the following:
```c++
#include <config_utilities/logging/log_to_stdout.h> // or your preferred logger
#include <config_utilities/settings.h>

config::Settings().external_libraries.log_allocation = true;
```

You can also disable loading external libraries by doing the following:
```c++
config::Settings().external_libraries.enabled = false;
```

Finally, we intentionally print to stderr when a library is being unloaded.
You can turn this behavior off by default by doing
```c++
config::Settings().external_libraries.verbose_load = false;
```
