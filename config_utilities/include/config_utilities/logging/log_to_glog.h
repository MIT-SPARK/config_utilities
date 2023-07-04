#pragma once

#include <memory>
#include <string>
#include <utility>

#include <glog/logging.h>

#include "config_utilities/factory.h"
#include "config_utilities/internal/logger.h"

namespace config::internal {

/**
 * @brief Implements logging to glog. This file pulls in glog as a dependency, but glog is not required if this file is
 * not included in the project.
 */
class GlogLogger : public Logger {
 protected:
  void log(const Severity severity, const std::string& message) override {
    // Default logs to std::cout to always have some sort of output. This could also be moved out to a separate logger
    // if we want this to be independent of iostream.
    if (severity == Severity::kFatal) {
      throw std::runtime_error(message);
    }
    switch (severity) {
      case Severity::kInfo:
        LOG(INFO) << message;
        break;

      case Severity::kWarning:
        LOG(WARNING) << message;

        break;

      case Severity::kError:
        LOG(ERROR) << message;
        break;

      case Severity::kFatal:
        LOG(FATAL) << message;
    }
  }

 private:
  // Factory registration to allow setting of formatters via Settings::setDefaultLogger().
  inline static const auto registration_ = Registration<Logger, GlogLogger>("glog");

  // Initialize the glog logger to be used if included.
  inline static const struct Initializer {
    Initializer() { Logger::setLogger(std::make_shared<GlogLogger>()); }
  } initializer_;
};

}  // namespace config::internal
