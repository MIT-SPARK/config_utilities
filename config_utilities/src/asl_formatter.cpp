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
  return what + " '" + data.name + "':\n" + internal::printCenter(data.name, print_width, '=') + "\n" +
         formatErrorsInternal(data, sev, print_width) + std::string(print_width, '=');
}

std::string AslFormatter::formatErrorsInternal(const MetaData& data,
                                               const std::string& sev,
                                               const size_t length) const {
  std::string warning;
  for (const std::string& error : data.errors) {
    warning.append(wrapString(sev + error, sev.length(), length, false) + "\n");
  }

  // Iterate through all sub-configs and append their errors.
  for (const MetaData& sub_config : data.sub_configs) {
    warning += formatErrorsInternal(sub_config, sev, length);
  }
  return warning;
}

std::string AslFormatter::formatToStringImpl(const MetaData& data) {
  std::string result = internal::printCenter(data.name, Settings::instance().print_width, '=') + "\n";
  // const int delta_indent = Settings::instance().subconfig_indent;
  // int indent = -delta_indent;
  // data.performOnAll([&](const MetaData& data) {
  //   indent += delta_indent;
  //   result += toStringInternal(data, indent);
  //   indent -= delta_indent;
  // });
  return result + std::string(Settings::instance().print_width, '=');
}

std::string AslFormatter::toStringInternal(const MetaData& data, size_t indent) const {
  std::string result;
  for (const FieldInfo& info : data.field_infos) {
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

  // Multi-line formatting info for nested types.
  std::vector<size_t> linebreaks = findAllSubstrings(field, "}, ");
  const std::vector<size_t> more_breaks = findAllSubstrings(field, "], ");
  linebreaks.insert(linebreaks.end(), more_breaks.begin(), more_breaks.end());
  std::vector<size_t> open_brackets, closed_brackets;
  const bool is_multiline = !linebreaks.empty();
  if (is_multiline) {
    std::sort(linebreaks.begin(), linebreaks.end());
    const std::vector<size_t> open_1 = findAllSubstrings(field, "{");
    const std::vector<size_t> open_2 = findAllSubstrings(field, "[");
    open_brackets.insert(open_brackets.end(), open_1.begin(), open_1.end());
    open_brackets.insert(open_brackets.end(), open_2.begin(), open_2.end());
    const std::vector<size_t> closed_1 = findAllSubstrings(field, "}");
    const std::vector<size_t> closed_2 = findAllSubstrings(field, "]");
    closed_brackets.insert(closed_brackets.end(), closed_1.begin(), closed_1.end());
    closed_brackets.insert(closed_brackets.end(), closed_2.begin(), closed_2.end());
    std::sort(open_brackets.begin(), open_brackets.end());
    std::sort(closed_brackets.begin(), closed_brackets.end());
  }

  // Format the header to width.
  result += wrapString(header, indent, print_width);
  const size_t last_header_line = result.find_last_of('\n');
  size_t header_size = result.substr(last_header_line != std::string::npos ? last_header_line + 1 : 0).size();
  if (header_size < global_indent) {
    result += std::string(global_indent - header_size, ' ');
    header_size = global_indent;
  } else if (print_width - header_size < field.length() || is_multiline) {
    // If the field does not fit entirely or is multi-line anyways just start a new line.
    result += "\n" + std::string(global_indent, ' ');
    header_size = global_indent;
  }

  // First line of field could be shorter due to header over extension.
  const size_t available_length = print_width - header_size;
  if (is_multiline) {
    // Multiline fields need formatting but start at new lines anyways.
    size_t prev_break = 0;
    linebreaks.push_back(field.length());
    for (size_t linebreak : linebreaks) {
      const auto isBefore = [prev_break](size_t i) { return i < prev_break; };
      size_t num_open = std::count_if(open_brackets.begin(), open_brackets.end(), isBefore) -
                        std::count_if(closed_brackets.begin(), closed_brackets.end(), isBefore);
      std::string line = field.substr(prev_break, linebreak - prev_break + 2);
      line = std::string(num_open, ' ') + line;
      line = wrapString(line, global_indent, print_width);
      if (prev_break == 0) {
        line = line.substr(global_indent);
      }
      result += line + "\n";
      prev_break = linebreak + 3;
    }
  } else if (field.length() < available_length) {
    // Field fits on one line.
    result += field + "\n";
  } else {
    // Add as much as fits on the first line and fill the rest.
    result += field.substr(0, available_length) + "\n";
    result += wrapString(field.substr(available_length), global_indent, print_width) + "\n";
  }
  return result;
}

std::string AslFormatter::wrapString(const std::string& str,
                                     size_t indent,
                                     size_t width,
                                     bool indent_first_line) const {
  std::string result;
  std::string remaining = str;
  if (indent_first_line) {
    result = std::string(indent, ' ');
  } else {
    const size_t first_line_length = std::min(indent, str.length());
    result = str.substr(0, first_line_length);
    remaining = str.substr(first_line_length);
  }
  const size_t length = width - indent;
  while (remaining.length() > length) {
    result += remaining.substr(0, length) + "\n" + std::string(indent, ' ');
    remaining = remaining.substr(length);
  }
  return result + remaining;
}

}  // namespace config::internal
