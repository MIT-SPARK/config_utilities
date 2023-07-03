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
 protected:
  void log(const Severity severity, const std::string& message) const override {
    // Default logs to std::cout to always have some sort of output. This could also be moved out to a separate logger
    // if we want this to be independent of iostream.
    if (severity == Severity::kFatal) {
      throw std::runtime_error(message);
    }
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
  // Factory registration to allow setting of formatters via Settings::setDefaultLogger().
  inline static const auto registration_ = Registration<Logger, StdoutLogger>("stdout");

  // Initialize the stdout logger to be used if included.
  inline static const struct Initializer {
    Initializer() { Logger::setDefaultLogger(std::make_unique<StdoutLogger>()); }
  } initializer_;
};

}  // namespace config::internal
