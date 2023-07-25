#include "config_utilities/formatting/asl.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/settings.h"

namespace config::internal {

std::string AslFormatter::formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity) {
  const std::string sev = severityToString(severity) + ": ";
  const size_t print_width = Settings::instance().print_width;
  is_first_divider_ = true;
  name_prefix_ = "";
  current_check_ = 0;
  if (indicate_num_checks_ && Settings::instance().inline_subconfig_field_names) {
    total_num_checks_ = 0;
    data.performOnAll([this](const MetaData& data) { total_num_checks_ += data.checks.size(); });
  }

  // Header line.
  std::string result = what + " '" + resolveConfigName(data) + "':\n" +
                       internal::printCenter(resolveConfigName(data), print_width, '=') + "\n";

  // Format all checks and errors.
  result += formatErrorsRecursive(data, sev, print_width);
  return result + std::string(print_width, '=');
}

std::string AslFormatter::formatErrorsRecursive(const MetaData& data, const std::string& sev, const size_t length) {
  const std::string name_prefix_before = name_prefix_;
  if (Settings::instance().inline_subconfig_field_names) {
    if (!data.field_name.empty()) {
      name_prefix_ += data.field_name + ".";
    }
  } else {
    current_check_ = 0;
    total_num_checks_ = data.checks.size();
  }
  std::string result = formatChecksInternal(data, sev, length) + formatErrorsInternal(data, sev, length);

  // Add more dividers if necessary.
  if (!Settings::instance().inline_subconfig_field_names && !result.empty()) {
    if (is_first_divider_) {
      is_first_divider_ = false;
    } else {
      result = internal::printCenter(resolveConfigName(data), Settings::instance().print_width, '-') + "\n" + result;
    }
  }

  for (const MetaData& sub_data : data.sub_configs) {
    result += formatErrorsRecursive(sub_data, sev, length);
  }
  name_prefix_ = name_prefix_before;
  return result;
}

std::string AslFormatter::formatChecksInternal(const MetaData& data, const std::string& sev, const size_t length) {
  std::string result;
  for (const auto& check : data.checks) {
    current_check_++;
    if (check->valid()) {
      continue;
    }
    const std::string rendered_name = check->name().empty() ? "" : " for '" + name_prefix_ + check->name() + "'";
    const std::string rendered_num =
        indicate_num_checks_ ? "[" + std::to_string(current_check_) + "/" + std::to_string(total_num_checks_) + "] "
                             : "";
    const std::string msg = sev + "Check " + rendered_num + "failed" + rendered_name + ": " + check->message() + ".";
    result.append(wrapString(msg, length, sev.length(), false) + "\n");
  }
  return result;
}

std::string AslFormatter::formatErrorsInternal(const MetaData& data, const std::string& sev, const size_t length) {
  std::string result;
  if (data.errors.empty()) {
    return result;
  }
  for (const std::string& error : data.errors) {
    result.append(wrapString(sev + error, length, sev.length(), false) + "\n");
  }
  return result;
}

std::string AslFormatter::formatConfigImpl(const MetaData& data) {
  return internal::printCenter(resolveConfigName(data), Settings::instance().print_width, '=') + "\n" +
         toStringInternal(data, 0) + std::string(Settings::instance().print_width, '=');
}

std::string AslFormatter::formatConfigsImpl(const std::vector<MetaData>& data) {
  if (data.empty()) {
    return "";
  }
  std::string result;
  for (const MetaData& config : data) {
    std::string entry = formatConfigImpl(config);
    entry.erase(entry.find_last_of("\n"));
    result += entry + "\n";
  }
  result += std::string(Settings::instance().print_width, '=');
  return result;
}

std::string AslFormatter::toStringInternal(const MetaData& data, size_t indent) const {
  std::string result;
  for (const FieldInfo& info : data.field_infos) {
    result += formatField(info, indent);
  }
  for (const MetaData& sub_data : data.sub_configs) {
    result += formatSubconfig(sub_data, indent);
  }
  return result;
}

std::string AslFormatter::formatSubconfig(const MetaData& data, size_t indent) const {
  // Header.
  std::string header = std::string(indent, ' ') + data.field_name;
  if (indicate_subconfig_types_) {
    header += " [" + resolveConfigName(data) + "]";
  }
  if (Settings::instance().indicate_default_values && indicate_subconfig_default_ && !data.is_virtual_config) {
    bool is_default = true;
    for (const FieldInfo& info : data.field_infos) {
      if (!info.is_default) {
        is_default = false;
        break;
      }
    }
    if (is_default) {
      header += " (default)";
    }
  }
  if (resolveConfigName(data) != "Uninitialized Virtual Config") {
    header += ":";
  }
  header += "\n";
  return header + toStringInternal(data, indent + Settings::instance().subconfig_indent);
}

std::string AslFormatter::formatField(const FieldInfo& info, size_t indent) const {
  std::string result;
  const size_t print_width = Settings::instance().print_width;
  const size_t global_indent = Settings::instance().print_indent;

  // field is the stringified value, The header is the field name.
  std::string field = dataToString(info.value);
  if (info.is_default && Settings::instance().indicate_default_values) {
    field += " (default)";
  }
  std::string header = std::string(indent, ' ') + info.name;
  if (Settings::instance().indicate_units && !info.unit.empty()) {
    header += " [" + info.unit + "]";
  }
  header += ":";

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
  result += wrapString(header, print_width, indent, false);
  const size_t last_header_line = result.find_last_of('\n');
  size_t header_size = result.substr(last_header_line != std::string::npos ? last_header_line + 1 : 0).size();
  if (header_size < global_indent) {
    result += std::string(global_indent - header_size, ' ');
    header_size = global_indent;
  } else if (print_width - header_size - 1 < field.length() || is_multiline) {
    // If the field does not fit entirely or is multi-line anyways just start a new line.
    result = pruneTrailingWhitespace(result);
    result += "\n" + std::string(global_indent, ' ');
    header_size = global_indent;
  } else {
    // If the field fits partly on the same line, add a space after the header.
    result += " ";
    header_size += 1;
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
      line = wrapString(line, print_width, global_indent) + "\n";
      if (prev_break == 0) {
        line = line.substr(global_indent);
      }
      result += pruneTrailingWhitespace(line);
      prev_break = linebreak + 3;
    }
  } else if (field.length() < available_length) {
    // Field fits on one line.
    result += pruneTrailingWhitespace(field) + "\n";
  } else {
    // Add as much as fits on the first line and fill the rest.
    result += pruneTrailingWhitespace(field.substr(0, available_length)) + "\n";
    result += wrapString(pruneLeadingWhitespace(field.substr(available_length)), print_width, global_indent) + "\n";
  }
  return result;
}

std::string AslFormatter::resolveConfigName(const MetaData& data) const {
  if (data.name.empty()) {
    if (data.is_virtual_config) {
      return "Uninitialized Virtual Config";
    } else {
      return "Unnamed Config";
    }
  } else {
    if (data.is_virtual_config && indicate_virtual_configs_) {
      return "Virtual Config: " + data.name;
    } else {
      return data.name;
    }
  }
}

}  // namespace config::internal
