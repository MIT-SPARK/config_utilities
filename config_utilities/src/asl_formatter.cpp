#include "config_utilities/formatting/asl.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/settings.h"

namespace config::internal {

std::string AslFormatter::formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity) {
  std::string sev;
  switch (severity) {
    case kWarning:
      sev = "Warning: ";
      break;
    case kError:
      sev = "Error: ";
      break;
    case kFatal:
      sev = "Fatal: ";
      break;
  }
  const size_t print_width = Settings::instance().print_width;
  const size_t length = print_width - sev.length();
  std::string warning = what + " '" + data.name + "':\n" + internal::printCenter(data.name, print_width, '=');
  for (std::string error : data.errors) {
    std::string line = sev;
    while (error.length() > length) {
      line.append(error.substr(0, length));
      error = error.substr(length);
      warning.append("\n" + line);
      line = std::string(sev.length(), ' ');
    }
    warning.append("\n" + line + error);
  }
  warning = warning + "\n" + std::string(print_width, '=');
  return warning;
}

std::string AslFormatter::formatToStringImpl(const MetaData& data) {
  return internal::printCenter(data.name, Settings::instance().print_width, '=') + "\n" + toStringInternal(data, 0) +
         std::string(Settings::instance().print_width, '=');
}

std::string AslFormatter::toStringInternal(const MetaData& data, size_t indent) const {
  std::string result;
  for (const FieldInfo& info : data.field_info) {
    result += formatField(data.data[info.name], info, indent);
  }
  return result;
}
std::string AslFormatter::formatField(const ConfigData& data, const FieldInfo& info, size_t indent) const {
  std::string result;
  const size_t print_width = Settings::instance().print_width;
  const size_t global_indent = Settings::instance().print_indent;

  // field is the stringified value, The header is the field name.
  std::string field = dataToString(data);
  if (info.is_default) {
    field += " (default)";
  }
  std::string header = std::string(indent, ' ') + info.name;
  if (Settings::instance().indicate_units && !info.unit.empty()) {
    header += " [" + info.unit + "]";
  }
  header += ": ";

  // Format the header to width.
  while (header.length() > print_width) {
    // Linebreaks for too long lines.
    result += header.substr(0, print_width) + "\n";
    header = header.substr(print_width);
  }
  if (header.length() < global_indent) {
    header.append(std::string(global_indent - header.length(), ' '));
  } else if (print_width - header.length() < field.length()) {
    result += header + "\n";
    header = std::string(global_indent, ' ');
  }

  // First line could be shorter.
  size_t length = print_width - header.length();
  if (field.length() > length) {
    result += header + field.substr(0, length) + "\n";
    field = field.substr(length);

    // Fill the rest.
    length = print_width - global_indent;
    while (field.length() > length) {
      result += std::string(global_indent, ' ') + field.substr(0, length) + "\n";
      field = field.substr(length);
    }
    result += std::string(global_indent, ' ') + field.substr(0, length) + "\n";
  } else {
    result += header + field + "\n";
  }
  return result;
}

}  // namespace config::internal
