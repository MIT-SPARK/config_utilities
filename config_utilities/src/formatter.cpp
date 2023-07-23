
#include "config_utilities/internal/formatter.h"

namespace config::internal {

std::string Formatter::formatErrors(const MetaData& data, const std::string& what, const Severity severity) {
  return formatter_->formatErrorsImpl(data, what, severity);
}

std::string Formatter::formatConfig(const MetaData& data) { return formatter_->formatConfigImpl(data); }

std::string Formatter::formatConfigs(const std::vector<MetaData>& data) { return formatter_->formatConfigsImpl(data); }

void Formatter::setFormatter(Formatter::Ptr formatter) {
  if (formatter) {
    formatter_ = std::move(formatter);
  }
}

std::string Formatter::formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity) {
  return "No format specified. Specify a format by including one of 'config_utilities/formatters/<preferred_style>.h'.";
}

std::string Formatter::formatConfigImpl(const MetaData& data) {
  return "No format specified. Specify a format by including one of 'config_utilities/formatters/<preferred_style>.h'.";
}

std::string Formatter::formatConfigsImpl(const std::vector<MetaData>& data) {
  return "No format specified. Specify a format by including one of 'config_utilities/formatters/<preferred_style>.h'.";
}

}  // namespace config::internal
