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
  enum class Severity { kInfo, kWarning, kError, kFatal };

  static void log(const Severity severity, const std::string& message) {
    if (instance_) {
      instance_->logImpl(severity, message);
    }
  }

  static void setLogger(std::shared_ptr<Logger> instance) { instance_ = std::move(instance); }

  // Convenience interfaces.
  static void logInfo(const std::string& message) { log(Severity::kInfo, message); }
  static void logWarning(const std::string& message) { log(Severity::kWarning, message); }
  static void logError(const std::string& message) { log(Severity::kError, message); }
  static void logFatal(const std::string& message) { log(Severity::kFatal, message); }

 protected:
  virtual void logImpl(const Severity severity, const std::string& message) = 0;

 private:
  inline static std::shared_ptr<Logger> instance_;
};

}  // namespace config::internal
