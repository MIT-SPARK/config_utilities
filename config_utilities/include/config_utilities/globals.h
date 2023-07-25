#pragma once

#include <string>
#include <vector>

#include "config_utilities/internal/meta_data.h"

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
  std::vector<MetaData> valid_configs;

 private:
  Globals() = default;
};

}  // namespace internal

// Access function in regular namespace.

/**
 * @brief Return a printed string of all configs that were checked valid (as a proxy for all configs that were created).
 * @param clear If true, clear the list of valid configs after printing.
 * @returns The formatted string of all validated configs so far.
 */
std::string printAllValidConfigs(bool clear = false);

}  // namespace config
