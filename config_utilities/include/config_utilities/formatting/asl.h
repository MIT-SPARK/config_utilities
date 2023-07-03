#pragma once

#include <memory>
#include <string>

#include "config_utilities/factory.h"
#include "config_utilities/internal/formatter.h"

namespace config::internal {

/**
 * @brief Implements formatting of text in the style of https://github.com/ethz-asl/config_utilities, emphasizing
 * readability wenn printed to the console.
 */
class AslFormatter : public Formatter {
 public:
  std::string formatCheckWarnings(const MetaData& data, Logger::Ptr logger = Logger::defaultLogger()) const override;
  std::string formatToString(const MetaData& data, Logger::Ptr logger = Logger::defaultLogger()) const override;

 private:
  // Factory registration to allow setting of formatters via Settings::setDefaultFormatter().
  inline static const auto registration_ = Registration<Formatter, AslFormatter>("asl");

  // Initialize the asl formatter to be used if included.
  inline static const struct Initializer {
    Initializer() { Formatter::setDefaultFormatter(std::make_unique<AslFormatter>()); }
  } initializer_;

  // Helper functions.
  std::string formatField(const std::string& name,
                          const std::string& value,
                          const std::string& unit = "",
                          const bool is_default = false) const;
};

}  // namespace config::internal
