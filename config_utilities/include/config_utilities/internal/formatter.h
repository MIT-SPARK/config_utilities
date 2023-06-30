#pragma once

#include <memory>
#include <string>
#include <utility>

#include "config_utilities/internal/meta_data.h"

namespace config::internal {

/**
 * @brief Abstract interface class for formatters. Formatters implement these methods to format the configs for
 * toString() calls and printing of validity checks.
 */
class Formatter {
 public:
  using Ptr = std::shared_ptr<Formatter>;
  using ConstPtr = std::shared_ptr<const Formatter>;

  Formatter() = default;
  virtual ~Formatter() = default;

  virtual std::string formatCheckWarnings(const MetaData& data) const {
    return "No formatter specified. Specify a format by including one of "
           "'config_utilities/formatters/<preferred_style>.h'.";
  }
  virtual std::string formatToString(const MetaData& data) const {
    return "No formatter specified. Specify a format by including one of "
           "'config_utilities/formatters/<preferred_style>.h'.";
  }

  static void setDefaultFormatter(Formatter::Ptr formatter) { default_formatter_ = std::move(formatter); }
  static const Formatter& defaultFormatter() { return *default_formatter_; }

 private:
  // Formatters need to implement this function returning a copy of themselves. Not specified pure virtual to allow
  // creation of empty formatters.
  virtual Formatter::Ptr clone() const { return std::make_shared<Formatter>(); }
  friend Formatter::Ptr optionalFormatter(const Formatter* const);
  inline static Formatter::ConstPtr default_formatter_ = std::make_shared<const Formatter>();
};

// Utility function for optionally specified formatters.
inline Formatter::Ptr optionalFormatter(const Formatter* const formatter) {
  return formatter ? formatter->clone() : Formatter::defaultFormatter().clone();
}

}  // namespace config::internal
