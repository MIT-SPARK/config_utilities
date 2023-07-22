#pragma once

#include <memory>
#include <string>
#include <vector>

#include "config_utilities/factory.h"
#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/meta_data.h"

namespace config::internal {

/**
 * @brief Implements formatting of text in the style of https://github.com/ethz-asl/config_utilities, emphasizing
 * readability wenn printed to the console.
 */
class AslFormatter : public Formatter {
 public:
  AslFormatter() = default;
  ~AslFormatter() override = default;

 protected:
  std::string formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity) override;
  std::string formatToStringImpl(const MetaData& data) override;
  std::string formatToStringImpl(const std::vector<MetaData>& data) override;

 private:
  // Factory registration to allow setting of formatters via Settings::setDefaultFormatter().
  inline static const auto registration_ = Registration<Formatter, AslFormatter>("asl");

  // Initialize the asl formatter to be used if included.
  inline static const struct Initializer {
    Initializer() { Formatter::setFormatter(std::make_unique<AslFormatter>()); }
  } initializer_;

  // Helper functions.
  std::string formatErrorsInternal(const MetaData& data, const std::string& sev, const size_t length) const;
  std::string toStringInternal(const MetaData& data, size_t indent) const;
  std::string formatField(const FieldInfo& info, size_t indent) const;
  std::string formatSubconfig(const MetaData& data, const FieldInfo& info, size_t indent) const;
  std::string wrapString(const std::string& str, size_t indent, size_t width, bool indent_first_line = true) const;
  std::string resolveConfigName(const MetaData& data) const;

  // Formatting options, currently hardcoded.
  constexpr static bool indicate_subconfig_types_ = true;
  constexpr static bool indicate_subconfig_default_ = true;
  constexpr static bool indicate_virtual_configs_ = true;
};

}  // namespace config::internal
