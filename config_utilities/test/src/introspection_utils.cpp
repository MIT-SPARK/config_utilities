#include "config_utilities/test/introspection_utils.h"

#include <filesystem>
#include <fstream>
#include <string>

#include "config_utilities/internal/introspection.h"
#include "config_utilities/settings.h"

namespace config::test {

void reset() {
  internal::Introspection::instance().clear();
  if (std::filesystem::exists(intro_dir)) {
    std::filesystem::remove_all(intro_dir);
  }
  Settings().introspection.output = intro_dir;
}

void disable() {
  reset();
  Settings().introspection.output.clear();
}

}  // namespace config::test