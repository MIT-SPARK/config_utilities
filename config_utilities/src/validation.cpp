#include "config_utilities/validation.h"

namespace config::internal {

bool hasNoInvalidChecks(const MetaData& data) {
  bool is_valid = true;
  data.performOnAll([&is_valid](const MetaData& d) {
    for (const auto& check : d.checks) {
      if (!check->valid() || !is_valid) {
        is_valid = false;
        return;
      }
    }
  });
  return is_valid;
}

}  // namespace config::internal
