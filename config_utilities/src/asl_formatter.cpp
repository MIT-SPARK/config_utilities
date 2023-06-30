#include "config_utilities/formatting/asl.h"
#include "config_utilities/internal/formatting_tools.h"
#include "config_utilities/settings.h"

namespace config::internal {

std::string AslFormatter::formatCheckWarnings(const MetaData& data) const {
  const std::string sev = "Warning: ";
  const size_t print_width = Settings::instance().print_width;
  const size_t length = print_width - sev.length();
  std::string warning = "Invalid config '" + data.name + "':\n" + internal::printCenter(data.name, print_width, '=');
  for (std::string w : data.warnings) {
    std::string line = sev;
    while (w.length() > length) {
      line.append(w.substr(0, length));
      w = w.substr(length);
      warning.append("\n" + line);
      line = std::string(sev.length(), ' ');
    }
    warning.append("\n" + line + w);
  }
  warning = warning + "\n" + std::string(print_width, '=');
  return warning;
}

std::string AslFormatter::formatToString(const MetaData& data) const { return std::string(); }

}  // namespace config::internal
