
#include "config_utilities/internal/formatter.h"

namespace config::internal {

std::string Formatter::formatErrors(const MetaData& data, const std::string& what, const Severity severity) {
  return formatter_->formatErrorsImpl(data, what, severity);
}

std::string Formatter::formatToString(const MetaData& data) { return formatter_->formatToStringImpl(data); }

std::string Formatter::formatToString(const std::vector<MetaData>& data) {
  return formatter_->formatToStringImpl(data);
}

void Formatter::setFormatter(Formatter::Ptr formatter) { formatter_ = std::move(formatter); }

std::string Formatter::formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity) {
  return "No format specified. Specify a format by including one of 'config_utilities/formatters/<preferred_style>.h'.";
}

std::string Formatter::formatToStringImpl(const MetaData& data) {
  return "No format specified. Specify a format by including one of 'config_utilities/formatters/<preferred_style>.h'.";
}

std::string Formatter::formatToStringImpl(const std::vector<MetaData>& data) {
  return "No format specified. Specify a format by including one of 'config_utilities/formatters/<preferred_style>.h'.";
}

}  // namespace config::internal
