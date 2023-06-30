#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include "config_utilities/internal/printing_tools.h"
#include "config_utilities/settings.h"

namespace config::internal {

/**
 * @brief Utility tool to make checking configs easier and more readable.
 * Instantiate the checker, then execute all checks, and get a summary of all
 * checks eventually.
 */
class ValidityChecker {
 public:
  ValidityChecker() { reset(); };
  ~ValidityChecker() = default;

  /**
   * @brief Return whether the config checker is valid, i.e. whether none of the
   * executed checks failed.
   *
   * @param print_warnings If true, print the warnings to console. Default:
   * false.
   */
  bool isValid(bool print_warnings = false) const {
    if (warnings_.empty()) {
      return true;
    }
    if (print_warnings) {
      const std::string sev = "Warning: ";
      const size_t length = print_width_ - sev.length();
      std::string warning = "\n" + internal::printCenter(name_, print_width_, '=');
      for (std::string w : warnings_) {
        std::string line = sev;
        while (w.length() > length) {
          line.append(w.substr(0, length));
          w = w.substr(length);
          warning.append("\n" + line);
          line = std::string(sev.length(), ' ');
        }
        warning.append("\n" + line + w);
      }
      warning = warning + "\n" + std::string(print_width_, '=');
      LOG(WARNING) << warning;
    }
    return false;
  }

  /**
   * @brief Enforce that the config is valid. This will terminate the program if
   * invalid.
   */
  void checkValid() const {
    if (!isValid(true)) {
      LOG(FATAL) << "Config '" << name_ << "' is not valid. See warnings above.";
    }
  }

  /**
   * @brief Execute a greater than (GT) check, i.e. param > value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkGT(const T& param, const T& value, const std::string& name) {
    if (param <= value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected > '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a greater equal (GE) check, i.e. param >= value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkGE(const T& param, const T& value, const std::string& name) {
    if (param < value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected >= '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a less than (LT) check, i.e. param < value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkLT(const T& param, const T& value, const std::string& name) {
    if (param >= value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected < '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a less equal (LE) check, i.e. param <= value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkLE(const T& param, const T& value, const std::string& name) {
    if (param > value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected <= '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute an equal (Eq) check, i.e. param == value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkEq(const T& param, const T& value, const std::string& name) {
    if (param != value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected to be '" << value << "' (is: '" << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a not equal (NE) check, i.e. param != value.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param value Value of the reference to compare to.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkNE(const T& param, const T& value, const std::string& name) {
    if (param == value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected to be different from '" << value << "'.";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a range check, i.e. lower <= param <= higher.
   *
   * @tparam T type of the parameter to be checked.
   * @param param Value of the parameter to be compared.
   * @param lower Lower bound of valid values.
   * @param higher Higher bound of valid values.
   * @param name Name of the parameter to be reported in the error summary.
   */
  template <typename T>
  void checkInRange(const T& param, const T& lower, const T& higher, const std::string& name) {
    if (param < lower || param > higher) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected to be within [" << lower << ", " << higher << "] (is: '" << param
         << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  /**
   * @brief Execute a condition check, i.e. whether condition is true.
   *
   * @param condition Condition that should evaluate to true if the config is valid.
   * @param warning Message to be reported in the error summary.
   */
  void checkCondition(bool condition, const std::string& warning) {
    if (!condition) {
      warnings_.emplace_back(warning);
    }
  }

  void setPrintWidth(int width) { print_width_ = width; }
  void setName(std::string name) { name_ = std::move(name); }
  void reset() {
    name_ = "Unnamed Config";
    print_width_ = Settings::instance().print_width;
    warnings_.clear();
  }

 private:
  std::string name_;
  int print_width_;
  std::vector<std::string> warnings_;
};

}  // namespace config::internal