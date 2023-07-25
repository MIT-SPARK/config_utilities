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
  std::string formatConfigImpl(const MetaData& data) override;
  std::string formatConfigsImpl(const std::vector<MetaData>& data) override;

 private:
  // Factory registration to allow setting of formatters via Settings::setDefaultFormatter().
  inline static const auto registration_ = Registration<Formatter, AslFormatter>("asl");

  // Initialize the asl formatter to be used if included.
  inline static const struct Initializer {
    Initializer() { Formatter::setFormatter(std::make_unique<AslFormatter>()); }
  } initializer_;

  // Helper functions.
  std::string formatErrorsRecursive(const MetaData& data, const std::string& sev, const size_t length);
  std::string formatChecksInternal(const MetaData& data, const std::string& sev, const size_t length);
  std::string formatErrorsInternal(const MetaData& data, const std::string& sev, const size_t length);
  std::string toStringInternal(const MetaData& data, size_t indent) const;
  std::string formatField(const FieldInfo& info, size_t indent) const;
  std::string formatSubconfig(const MetaData& data, size_t indent) const;
  std::string resolveConfigName(const MetaData& data) const;

  // Formatting options, currently not exposed in global settings but work if want changed.
  // TODO(lschmid): Global formatting options should probably be a config of the formatter.
  // If true add subconfig types after the fieldname.
  constexpr static bool indicate_subconfig_types_ = true;
  // If true label subconfigs as default if all their values are default.
  constexpr static bool indicate_subconfig_default_ = true;
  // If true indicate that a config is a virtual config in the config name.
  constexpr static bool indicate_virtual_configs_ = true;
  // If true indicate the number of a check and total number of checks in failed checks.
  constexpr static bool indicate_num_checks_ = true;

  // Variables.
  std::string name_prefix_;
  size_t total_num_checks_;
  size_t current_check_;
  bool is_first_divider_;
};

}  // namespace config::internal
