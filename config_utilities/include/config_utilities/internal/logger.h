#pragma once

#include <exception>
#include <memory>
#include <string>
#include <utility>

namespace config::internal {

// Enum for different severity levels of logging.
enum class Severity { kInfo, kWarning, kError, kFatal };

std::string severityToString(const Severity severity);

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
  static void log(const Severity severity, const std::string& message);

  // Convenience interfaces to log to a specific severity.
  static void logInfo(const std::string& message);
  static void logWarning(const std::string& message);
  static void logError(const std::string& message);
  static void logFatal(const std::string& message);

  // Set the global logger.
  static void setLogger(Logger::Ptr logger);

 protected:
  // Interface to be implemented by loggers.
  virtual void logImpl(const Severity severity, const std::string& message);

 private:
  inline static Logger::Ptr logger_ = std::make_shared<Logger>();
};

}  // namespace config::internal
