#include "config_utilities/globals.h"

#include "config_utilities/internal/formatter.h"

namespace config {

std::string printAllValidConfigs(bool clear) {
  const std::string result = internal::Formatter::formatConfigs(internal::Globals::instance().valid_configs);
  if (clear) {
    internal::Globals::instance().valid_configs.clear();
  }
  return result;
}

}  // namespace config
