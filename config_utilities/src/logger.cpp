#include "config_utilities/internal/logger.h"

namespace config::internal {

std::string severityToString(const Severity severity) {
  switch (severity) {
    case Severity::kInfo:
      return "Info";
    case Severity::kWarning:
      return "Warning";
    case Severity::kError:
      return "Error";
    case Severity::kFatal:
      return "Fatal";
  }
  return "UNKNOWN";
}

void Logger::log(const Severity severity, const std::string& message) { logger_->logImpl(severity, message); }

void Logger::logInfo(const std::string& message) { logger_->logImpl(Severity::kInfo, message); }

void Logger::logWarning(const std::string& message) { logger_->logImpl(Severity::kWarning, message); }

void Logger::logError(const std::string& message) { logger_->logImpl(Severity::kError, message); }

void Logger::logFatal(const std::string& message) { logger_->logImpl(Severity::kFatal, message); }

void Logger::setLogger(Logger::Ptr logger) {
  if (logger) {
    logger_ = std::move(logger);
  }
}

void Logger::logImpl(const Severity severity, const std::string& message) {
  // Empty logger does not log anything. Implementations of logFatal are expected to stop execution of the program.
  if (severity == Severity::kFatal) {
    throw std::runtime_error("FATAL: " + message);
  }
}

}  // namespace config::internal
