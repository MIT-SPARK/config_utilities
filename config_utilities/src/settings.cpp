#include "config_utilities/settings.h"

#include <memory>

#include "config_utilities/factory.h"
#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"

namespace config::internal {

Settings Settings::instance_;

Settings& Settings::instance() { return instance_; }

void Settings::setLogger(const std::string& name) {
  if (name == "none") {
    Logger::setLogger(std::make_unique<Logger>());
    return;
  }
  std::unique_ptr<Logger> new_logger = create<Logger>(name);
  if (new_logger) {
    Logger::setLogger(std::move(new_logger));
  }
}

void Settings::setFormatter(const std::string& name) {
  if (name == "none") {
    Formatter::setFormatter(std::make_unique<Formatter>());
    return;
  }
  std::unique_ptr<Formatter> new_formatter = create<Formatter>(name);
  if (new_formatter) {
    Formatter::setFormatter(std::move(new_formatter));
  }
}

}  // namespace config::internal
