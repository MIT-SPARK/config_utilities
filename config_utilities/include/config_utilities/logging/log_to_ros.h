#pragma once

#include <memory>
#include <string>
#include <utility>

#include <ros/console.h>

#include "config_utilities/factory.h"
#include "config_utilities/internal/logger.h"

namespace config::internal {

/**
 * @brief Implements logging to roslog. This file pulls in ros as a dependency, but its not required if this file is
 * not included in the project.
 */
class RosLogger : public Logger {
 public:
  RosLogger() = default;
  virtual ~RosLogger() = default;

 protected:
  void logImpl(const Severity severity, const std::string& message) override {
    switch (severity) {
      case Severity::kInfo:
        ROS_INFO_STREAM(message);
        break;

      case Severity::kWarning:
        ROS_WARN_STREAM(message);
        break;

      case Severity::kError:
        ROS_ERROR_STREAM(message);
        break;

      case Severity::kFatal:
        ROS_FATAL_STREAM(message);
    }
  }

 private:
  // Factory registration to allow setting of formatters via Settings::setLogger().
  inline static const auto registration_ = Registration<Logger, RosLogger>("ros");

  // Initialize the ros logger to be used if included.
  inline static const struct Initializer {
    Initializer() { Logger::setLogger(std::make_shared<RosLogger>()); }
  } initializer_;
};

}  // namespace config::internal
