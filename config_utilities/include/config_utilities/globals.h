#pragma once

#include <string>
#include <vector>

namespace config {

namespace internal {

/**
 * @brief Global globals for how config_utilities-based configs behave. These
 * can be dynamically set and changed throughout a program.
 */
struct Globals {
  Globals(const Globals& other) = delete;
  Globals(const Globals&& other) = delete;
  Globals& operator=(const Globals& other) = delete;
  Globals& operator=(const Globals&& other) = delete;

  // Singleton access to the global globals.
  static Globals& instance() {
    static Globals globals;
    return globals;
  }

  // Keep track of all configs that were checked valid (as a proxy for all configs that were created).
  std::vector<std::string> valid_configs;

 private:
  Globals() = default;
};

}  // namespace internal

// Access function in regular namespace.

/**
 * @brief Return a printed string of all configs that were checked valid (as a proxy for all configs that were created).
 */
inline std::vector<std::string> getValidConfigs() { return internal::Globals::instance().valid_configs; }

}  // namespace config
