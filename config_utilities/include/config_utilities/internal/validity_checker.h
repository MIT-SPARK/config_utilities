#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "config_utilities/checks.h"

namespace config::internal {

/**
 * @brief Utility tool to make checking configs easier and more readable. Instantiate the checker, then execute all
 * checks, and get a summary of all checks eventually. For every failed check a warning is added to the summary. If
 * there are no warnings this means that all checks have passed.
 *
 * NOTE(lschmid): The formatting of warnings could also be specified via a parser but for now just use this fixed
 * warning.
 */
class ValidityChecker {
 public:
  ValidityChecker() = default;
  ~ValidityChecker() = default;

  void setFieldNamePrefix(const std::string& prefix) { name_prefix_ = prefix; }

  void checkCondition(const CheckBase& check) {
    if (!check) {
      warnings_.emplace_back(check.toString(name_prefix_));
    }
  }

  void markFailedCheck(const std::string& message) { warnings_.emplace_back(message); }

  void resetWarnings() { warnings_.clear(); }
  std::vector<std::string>& warnings() { return warnings_; }
  const std::vector<std::string>& getWarnings() const { return warnings_; }

 private:
  std::vector<std::string> warnings_;
  std::string name_prefix_;  // Used to prefix field names in warnings.
};

}  // namespace config::internal
