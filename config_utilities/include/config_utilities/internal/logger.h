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
  using ConstPtr = std::shared_ptr<const Logger>;

  Logger() = default;
  virtual ~Logger() = default;

  // Severity levels for logging.
  enum class Severity { kInfo, kWarning, kError, kFatal };

  // Interface to be implemented by loggers.
  virtual void log(const Severity severity, const std::string& message) const {
    /* Empty logger does not log anything. */
  }

  // Convenience interfaces to log to a specific severity.
  void logInfo(const std::string& message) const { log(Severity::kInfo, message); }
  void logWarning(const std::string& message) const { log(Severity::kWarning, message); }
  void logError(const std::string& message) const { log(Severity::kError, message); }
  // Important: Implementations of logFatal are expected to stop execution of the program.
  void logFatal(const std::string& message) const { log(Severity::kFatal, message); }

  static void setDefaultLogger(Logger::Ptr logger) { default_logger_ = std::move(logger); }
  static const Logger& defaultLogger() { return *default_logger_; }

 private:
  // Loggers need to implement this function returning a copy of themselves. Not specified pure virtual to allow
  // creation of empty loggers.
  virtual Logger::Ptr clone() const { return std::make_shared<Logger>(); }
  friend Logger::Ptr optionalLogger(const Logger* const logger);
  inline static Logger::ConstPtr default_logger_ = std::shared_ptr<const Logger>();
};

// Utility function for optionally specified logger.
inline Logger::Ptr optionalLogger(const Logger* const logger) {
  return (logger ? logger->clone() : Logger::defaultLogger().clone());
}

}  // namespace config::internal
