#pragma once

#include <memory>
#include <string>
#include <utility>

namespace config::internal {

/**
 * @brief Abstract interface class for all logging classes. Calls to logging will happen through a global instance of
 * the logger that can be set by the implementation.
 */
class Logger {
 public:
  using Ptr = std::shared_ptr<Logger>;

  Logger() = default;
  virtual ~Logger() = default;

  // Convenience interfaces to log to a specific severity.
  static void logInfo(const std::string& message) { logger_->log(Severity::kInfo, message); }
  static void logWarning(const std::string& message) { logger_->log(Severity::kWarning, message); }
  static void logError(const std::string& message) { logger_->log(Severity::kError, message); }
  // Important: Implementations of logFatal are expected to stop execution of the program.
  static void logFatal(const std::string& message) { logger_->log(Severity::kFatal, message); }

  static void setLogger(Logger::Ptr logger) { logger_ = std::move(logger); }

 protected:
  // Severity levels for logging.
  enum class Severity { kInfo, kWarning, kError, kFatal };

  // Interface to be implemented by loggers.
  virtual void log(const Severity severity, const std::string& message) { /* Empty logger does not log anything. */
  }

 private:
  inline static Logger::Ptr logger_ = std::shared_ptr<Logger>();
};

}  // namespace config::internal
