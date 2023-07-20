#pragma once

#include <memory>
#include <string>
#include <utility>

namespace config::internal {

// Enum for different severity levels of logging.
enum class Severity { kInfo, kWarning, kError, kFatal };

inline std::string severityToString(const Severity severity) {
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

/**
 * @brief Abstract interface class for all logging classes. Calls to logging will happen through a global instance of
 * the logger that can be set by the implementation.
 */
class Logger {
 public:
  using Ptr = std::shared_ptr<Logger>;

  // Constructor and destructor.
  Logger() = default;
  virtual ~Logger() = default;

  // Severity levels for logging. Important: Implementations of logFatal are expected to stop execution of the program.
  static void log(const Severity severity, const std::string& message) { logger_->logImpl(severity, message); }

  // Convenience interfaces to log to a specific severity.
  static void logInfo(const std::string& message) { logger_->logImpl(Severity::kInfo, message); }
  static void logWarning(const std::string& message) { logger_->logImpl(Severity::kWarning, message); }
  static void logError(const std::string& message) { logger_->logImpl(Severity::kError, message); }
  static void logFatal(const std::string& message) { logger_->logImpl(Severity::kFatal, message); }

  // Set the global logger.
  static void setLogger(Logger::Ptr logger) { logger_ = std::move(logger); }

 protected:
  // Interface to be implemented by loggers.
  virtual void logImpl(const Severity severity, const std::string& message) { /* Empty logger does not log anything. */
  }

 private:
  inline static Logger::Ptr logger_ = std::shared_ptr<Logger>();
};

}  // namespace config::internal
