#include "config_utilities/settings.h"

#include <memory>

#include "config_utilities/factory.h"
#include "config_utilities/internal/logger.h"

namespace config::internal {

void Settings::setLogger(const std::string& name) {
  std::unique_ptr<Logger> new_logger = Factory::create<Logger>(name);
  if (new_logger) {
    Logger::setLogger(std::move(new_logger));
  }
}

}  // namespace config::internal
