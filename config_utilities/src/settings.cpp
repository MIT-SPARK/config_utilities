#include "config_utilities/settings.h"

#include <memory>

#include "config_utilities/factory.h"
#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"

namespace config::internal {

void Settings::setDefaultLogger(const std::string& name) {
  std::unique_ptr<Logger> new_logger = Factory::create<Logger>(name);
  if (new_logger) {
    Logger::setDefaultLogger(std::move(new_logger));
  }
}

void Settings::setDefaultFormatter(const std::string& name) {
  std::unique_ptr<Formatter> new_formatter = Factory::create<Formatter>(name);
  if (new_formatter) {
    Formatter::setDefaultFormatter(std::move(new_formatter));
  }
}

}  // namespace config::internal
