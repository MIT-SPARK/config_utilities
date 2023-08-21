#pragma once

#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include "config_utilities/factory.h"
#include "config_utilities/internal/logger.h"

namespace config::internal {

/**
 * @brief Implements logging to std::cout.
 */
class StdoutLogger : public Logger {
 public:
  StdoutLogger() = default;
  virtual ~StdoutLogger() = default;

 protected:
  void logImpl(const Severity severity, const std::string& message) override {
    switch (severity) {
      case Severity::kInfo:
        std::cout << "[INFO] " << message << std::endl;
        break;

      case Severity::kWarning:
        std::cout << "\033[33m[WARNING] " << message << "\033[0m" << std::endl;
        break;

      case Severity::kError:
        std::cout << "\033[31m[ERROR] " << message << "\033[0m" << std::endl;
        break;

      case Severity::kFatal:
        throw std::runtime_error(message);
    }
  }

 private:
  // Factory registration to allow setting of formatters via Settings::setLogger().
  inline static const auto registration_ = Registration<Logger, StdoutLogger>("stdout");

  // Initialize the stdout logger to be used if included.
  inline static const struct Initializer {
    Initializer() { Logger::setLogger(std::make_shared<StdoutLogger>()); }
  } initializer_;
};

}  // namespace config::internal
