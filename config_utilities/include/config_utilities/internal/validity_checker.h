#pragma once

#include <sstream>
#include <string>
#include <vector>

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

  template <typename T>
  void checkGT(const T& param, const T& value, const std::string& name) {
    if (param <= value) {
      std::stringstream ss;
      ss << "Param '" << name_prefix_ + name << "' is expected > '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkGE(const T& param, const T& value, const std::string& name) {
    if (param < value) {
      std::stringstream ss;
      ss << "Param '" << name_prefix_ + name << "' is expected >= '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkLT(const T& param, const T& value, const std::string& name) {
    if (param >= value) {
      std::stringstream ss;
      ss << "Param '" << name_prefix_ + name << "' is expected < '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkLE(const T& param, const T& value, const std::string& name) {
    if (param > value) {
      std::stringstream ss;
      ss << "Param '" << name_prefix_ + name << "' is expected <= '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkEq(const T& param, const T& value, const std::string& name) {
    if (param != value) {
      std::stringstream ss;
      ss << "Param '" << name_prefix_ + name << "' is expected to be '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkNE(const T& param, const T& value, const std::string& name) {
    if (param == value) {
      std::stringstream ss;
      ss << "Param '" << name_prefix_ + name << "' is expected to be different from '" << value << "'.";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkInRange(const T& param, const T& lower, const T& higher, const std::string& name) {
    if (param < lower || param > higher) {
      std::stringstream ss;
      ss << "Param '" << name_prefix_ + name << "' is expected to be within [" << lower << ", " << higher << "] (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  void checkCondition(bool condition, const std::string& warning) {
    if (!condition) {
      warnings_.emplace_back(warning);
    }
  }

  void resetWarnings() { warnings_.clear(); }
  std::vector<std::string>& warnings() { return warnings_; }
  const std::vector<std::string>& getWarnings() const { return warnings_; }

 private:
  std::vector<std::string> warnings_;
  std::string name_prefix_;  // Used to prefix field names in warnings.
};

}  // namespace config::internal
