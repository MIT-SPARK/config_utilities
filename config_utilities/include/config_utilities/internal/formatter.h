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

  // Accessing the formatter.
  static std::string formatErrors(const MetaData& data) { return formatter_->formatErrorsImpl(data); }
  static std::string formatToString(const MetaData& data) { return formatter_->formatToStringImpl(data); }

  static void setFormatter(Formatter::Ptr formatter) { formatter_ = std::move(formatter); }

 protected:
  virtual std::string formatErrorsImpl(const MetaData& data) {
    return "No formatter specified. Specify a format by including one of "
           "'config_utilities/formatters/<preferred_style>.h'.";
  }
  virtual std::string formatToStringImpl(const MetaData& data) {
    return "No formatter specified. Specify a format by including one of "
           "'config_utilities/formatters/<preferred_style>.h'.";
  }

 private:
  inline static Formatter::Ptr formatter_ = std::make_shared<Formatter>();
};

}  // namespace config::internal
