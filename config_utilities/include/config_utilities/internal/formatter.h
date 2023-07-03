#pragma once

#include <memory>
#include <string>
#include <utility>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/meta_data.h"

namespace config::internal {

/**
 * @brief Abstract interface class for formatters. Formatters implement these methods to format the configs for
 * toString() calls and printing of validity checks.
 */
class Formatter {
 public:
  using Ptr = std::shared_ptr<Formatter>;

  Formatter() = default;
  virtual ~Formatter() = default;

  virtual std::string formatCheckWarnings(const MetaData& data, Logger::Ptr logger = Logger::defaultLogger()) const {
    return "No formatter specified. Specify a format by including one of "
           "'config_utilities/formatters/<preferred_style>.h'.";
  }
  virtual std::string formatToString(const MetaData& data, Logger::Ptr logger = Logger::defaultLogger()) const {
    return "No formatter specified. Specify a format by including one of "
           "'config_utilities/formatters/<preferred_style>.h'.";
  }

  static void setDefaultFormatter(Formatter::Ptr formatter) { default_formatter_ = std::move(formatter); }
  static Formatter::Ptr defaultFormatter() { return default_formatter_; }

 private:
  inline static Formatter::Ptr default_formatter_ = std::make_shared<Formatter>();
};

}  // namespace config::internal
