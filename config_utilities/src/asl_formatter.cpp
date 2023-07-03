#include "config_utilities/formatting/asl.h"
#include "config_utilities/internal/formatting_tools.h"
#include "config_utilities/settings.h"

namespace config::internal {

std::string AslFormatter::formatCheckWarnings(const MetaData& data, Logger::Ptr /* logger */) const {
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

std::string AslFormatter::formatToString(const MetaData& data, Logger::Ptr logger) const {
  std::string result = internal::printCenter(data.name, Settings::instance().print_width, '=') + "\n";
  // toStringInternal(meta_data_->indent) + "\n" + std::string(Settings::instance().print_width, '=');
  return result;
}

std::string AslFormatter::formatField(const std::string& name,
                                      const std::string& value,
                                      const std::string& unit,
                                      const bool is_default) const {
  // std::string f = field;
  // if (meta_data_->default_values) {
  //   auto it = meta_data_->default_values->find(name);
  //   if (it != meta_data_->default_values->end()) {
  //     if (it->second == field) {
  //       f.append(" (default)");
  //     }
  //   }
  // }

  // // The header is the field name.
  // std::string header = std::string(meta_data_->indent, ' ') + name;
  // if (GlobalSettings::instance().indicate_units && !unit.empty()) {
  //   header += " [" + unit + "]: ";
  // } else {
  //   header += ": ";
  // }
  // while (header.length() > GlobalSettings::instance().print_width) {
  //   // Linebreaks for too long lines.
  //   meta_data_->messages->emplace_back(header.substr(0, GlobalSettings::instance().print_width));
  //   header = header.substr(GlobalSettings::instance().print_width);
  // }
  // if (header.length() < GlobalSettings::instance().print_indent) {
  //   header.append(std::string(GlobalSettings::instance().print_indent - header.length(), ' '));
  // } else if (GlobalSettings::instance().print_width - header.length() < f.length()) {
  //   meta_data_->messages->emplace_back(header);
  //   header = std::string(GlobalSettings::instance().print_indent, ' ');
  // }

  // // First line could be shorter.
  // size_t length = GlobalSettings::instance().print_width - header.length();
  // if (f.length() > length) {
  //   meta_data_->messages->emplace_back(header + f.substr(0, length));
  //   f = f.substr(length);

  //   // Fill the rest.
  //   length = GlobalSettings::instance().print_width - GlobalSettings::instance().print_indent;
  //   while (f.length() > length) {
  //     meta_data_->messages->emplace_back(std::string(GlobalSettings::instance().print_indent, ' ') +
  //                                        f.substr(0, length));
  //     f = f.substr(length);
  //   }
  //   meta_data_->messages->emplace_back(std::string(GlobalSettings::instance().print_indent, ' ') + f.substr(0,
  //   length));
  // } else {
  //   meta_data_->messages->emplace_back(header.append(f));
  // }
}

// Function that produces the a vector of strings for each field of a config.
// template <typename ConfigT>
// std::string toStringInternal(const ConfigT& config, int indent) {
//   const int indent_prev = meta_data_->indent;
//   meta_data_->indent = indent;

//   meta_data_->messages = std::make_unique<std::vector<std::string>>();

//   // Create the default values if required.
//   if (Settings::instance().indicate_default_values) {
//     ConfigT defaults;
//     Visitor& data = Visitor::instance();
//     data.get_defaults = true;

//     meta_data_->default_values = std::make_unique<std::unordered_map<std::string,
//     std::string>>(defaults->getValues()); meta_data_->use_printing_to_get_values = true;
//     meta_data_->merged_setup_already_used = true;

//     // NOTE: setupParamsAndPrinting() does not modify 'this' in printing mode.
//     ((ConfigInternal*)this)->setupParamsAndPrinting();
//     printFields();
//   }

//   meta_data_->use_printing_to_get_values = false;
//   meta_data_->merged_setup_already_used = true;

//   // NOTE: setupParamsAndPrinting() does not modify 'this' in printing mode.
//   ((ConfigInternal*)this)->setupParamsAndPrinting();
//   printFields();
//   std::string result;
//   for (const std::string& msg : *(meta_data_->messages)) {
//     result.append("\n" + msg);
//   }
//   if (!result.empty()) {
//     result = result.substr(1);
//   }
//   meta_data_->messages.reset(nullptr);
//   meta_data_->default_values.reset(nullptr);
//   meta_data_->indent = indent_prev;
//   meta_data_->merged_setup_currently_active = false;
//   meta_data_->global_printing_processed = true;
//   return result;
// };

/**
 * @brief Print this parameter in the config toString summary. The parameter
 * type T is required to implement the << operator.
 *
 * @tparam T Type of the parameter to be printed.
 * @param name Name of the parameter to be displayed.
 * @param field Value of the parameter to be retrieved.
 * @param unit Unit of the parameter. If not set nothing will be displayed.
 */
// template <typename T>
// void printField(const std::string& name,
//                 const T& field,
//                 const std::string& unit = "") const {
//   if (isConfig(&field)) {
//     if (!meta_data_->use_printing_to_get_values) {
//       // The default values only apply for non-config fields, as these will be
//       // managed internally when printing the sub-config.
//       printConfigInternal(name, (const internal::ConfigInternal*)&field);
//     }
//   } else if (isVariableConfig(&field)) {
//     if (!meta_data_->use_printing_to_get_values) {
//       // The default values only apply for non-config fields, as these will be
//       // managed internally when printing the sub-config.
//       auto var = (const internal::VariableConfigInternal*)&field;
//       if (var->config_ != nullptr) {
//         printConfigInternal(
//             name + " (Variable Config, type '" + var->type_ + "')",
//             var->config_.get());

//       } else {
//         printFieldInternal(name, "Uninitialized Variable Config.");
//       }
//     }
//   } else {
//     std::stringstream ss;
//     ss << field;
//     printFieldInternal(name, ss.str(), unit);
//   }
// }

}  // namespace config::internal
