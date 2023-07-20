#include <exception>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config::internal {

// Interfaces for all internal tools interact with configs through the visitor.
template <typename ConfigT>
MetaData Visitor::setValues(ConfigT& config, const YAML::Node& node, bool print_warnings) {
  Visitor visitor = Visitor::create();
  visitor.parser.node() = node;
  visitor.mode = Visitor::Mode::kSet;
  declare_config(config);
  visitor.extractErrors();

  if (print_warnings && visitor.data.hasErrors()) {
    Logger::logWarning(Formatter::formatErrors(visitor.data, "Errors parsing config", Severity::kWarning));
  }
  return visitor.data;
}

template <typename ConfigT>
MetaData Visitor::getValues(const ConfigT& config, bool print_warnings) {
  Visitor visitor = Visitor::create();
  visitor.mode = Visitor::Mode::kGet;
  // NOTE: We know that in mode kGet, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));
  visitor.extractErrors();
  visitor.data.data = visitor.parser.node();

  if (print_warnings && visitor.data.hasErrors()) {
    Logger::logWarning(Formatter::formatErrors(visitor.data, "Errors parsing config", Severity::kWarning));
  }
  return visitor.data;
}

template <typename ConfigT>
MetaData Visitor::getChecks(const ConfigT& config) {
  Visitor visitor = Visitor::create();
  visitor.mode = Visitor::Mode::kCheck;
  // NOTE: We know that in mode kCheck, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));
  visitor.extractErrors();

  return visitor.data;
}

template <typename T>
void Visitor::visitField(T& field, const std::string& field_name, const std::string& unit) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode == Visitor::Mode::kSet) {
    visitor.parser.fromYaml(field_name, field, visitor.name_space, visitor.name_prefix);
  }

  if (visitor.mode == Visitor::Mode::kGet) {
    FieldInfo& info = visitor.data.field_infos.emplace_back();
    visitor.parser.toYaml(field_name, field, visitor.name_space, visitor.name_prefix);
    info.name = field_name;
    info.unit = unit;
  }
}

template <typename EnumT>
void Visitor::visitEnumField(EnumT& field,
                             const std::string& field_name,
                             const std::map<EnumT, std::string>& enum_names) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kSet && visitor.mode != Visitor::Mode::kGet) {
    return;
  }

  // Check uniqueness of enum names.
  std::set<std::string> names;
  for (const auto& [_, name] : enum_names) {
    if (names.find(name) != names.end()) {
      visitor.data.errors.emplace_back("Value '" + name + "' is defined multiple times for enum field '" + field_name +
                                       "'.");
    }
    names.insert(name);
  }

  // Parse enums. These are internally stored as strings.
  if (visitor.mode == Visitor::Mode::kSet) {
    std::string place_holder;
    if (visitor.parser.fromYaml(field_name, place_holder, visitor.name_space, visitor.name_prefix)) {
      const auto it = std::find_if(enum_names.begin(), enum_names.end(), [&place_holder](const auto& pair) {
        return pair.second == place_holder;
      });
      if (it == enum_names.end()) {
        std::string error = "Failed to parse param '" + field_name + "': Invalid value '" + place_holder +
                            "' for enum field with values ";
        for (const auto& [_, name] : enum_names) {
          error += "'" + name + "', ";
        }
        visitor.data.errors.emplace_back(error.substr(0, error.size() - 2) + ".");
      } else {
        field = it->first;
      }
    }
  }

  if (visitor.mode == Visitor::Mode::kGet) {
    FieldInfo& info = visitor.data.field_infos.emplace_back();
    const auto it = enum_names.find(field);
    if (it == enum_names.end()) {
      visitor.data.errors.emplace_back("Value of enum field '" + field_name + "' is out of the defined range.");
      visitor.parser.toYaml(field_name, "INVALID ENUM VALUE", visitor.name_space, visitor.name_prefix);
    } else {
      visitor.parser.toYaml(field_name, it->second, visitor.name_space, visitor.name_prefix);
    }
    info.name = field_name;
  }
}

template <typename T>
void Visitor::visitCheck(Visitor::CheckMode mode, const T& param, const T& value, const std::string& name) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }

  switch (mode) {
    case Visitor::CheckMode::kGT:
      visitor.checker.checkGT(param, value, name);
      return;
    case Visitor::CheckMode::kGE:
      visitor.checker.checkGE(param, value, name);
      return;
    case Visitor::CheckMode::kLT:
      visitor.checker.checkLT(param, value, name);
      return;
    case Visitor::CheckMode::kLE:
      visitor.checker.checkLE(param, value, name);
      return;
    case Visitor::CheckMode::kEQ:
      visitor.checker.checkEq(param, value, name);
      return;
    case Visitor::CheckMode::kNE:
      visitor.checker.checkNE(param, value, name);
      return;
  }
}

template <typename T>
void Visitor::visitCheckInRange(const T& param, const T& lower, const T& upper, const std::string& name) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.checker.checkInRange(param, lower, upper, name);
}

template <typename ConfigT>
void Visitor::visitSubconfig(ConfigT& config, const std::string& field_name, const std::string& sub_namespace) {
  Visitor& visitor = Visitor::instance();

  if (visitor.mode != Visitor::Mode::kSet && visitor.mode != Visitor::Mode::kGet &&
      visitor.mode != Visitor::Mode::kCheck) {
    return;
  }

  // Check is a configT. Ceck this at runtime to allow more flexibility for config declaration.
  if (!isConfig<ConfigT>()) {
    visitor.data.errors.emplace_back("Subconfig field '" + field_name + "' has not beend declared a config.");
    return;
  }

  // Add the field info.
  FieldInfo& info = visitor.data.field_infos.emplace_back();
  info.name = field_name;
  info.subconfig_id = visitor.data.sub_configs.size();

  // Store state as was before.
  visitor.extractErrors();
  const MetaData data_before = visitor.data;
  const std::string name_space_before = visitor.name_space;
  const std::string name_prefix_before = visitor.name_prefix;

  // Set new data.
  if (Settings::instance().index_subconfig_field_names) {
    visitor.name_prefix += field_name + ".";
    visitor.checker.setFieldNamePrefix(visitor.name_prefix);
  }
  visitor.name_space = joinNamespace(visitor.name_space, sub_namespace);
  visitor.data = MetaData();
  visitor.data.current_field_name = field_name;

  // Visit subconfig.
  declare_config(config);

  // Aggregate data.
  visitor.extractErrors();
  MetaData data_after = visitor.data;
  if (visitor.mode == Visitor::Mode::kGet) {
    data_after.data = lookupNamespace(YAML::Clone(visitor.parser.node()), visitor.name_space);
  }

  // Restore state.
  visitor.name_space = name_space_before;
  visitor.name_prefix = name_prefix_before;
  visitor.checker.setFieldNamePrefix(name_prefix_before);
  visitor.data = data_before;
  visitor.data.sub_configs.emplace_back(std::move(data_after));
}

template <typename ConfigT>
void Visitor::visitBase(ConfigT& config, const std::string& sub_namespace) {
  Visitor& visitor = Visitor::instance();
  // For now simply pass the call to the base config, treating it as a single config object.
  // NOTE(lschmid): Alternatives are to treat it as a subconfig for clearer readability.
  const std::string name_space_before = visitor.name_space;
  const std::string name_before = visitor.data.name;
  visitor.name_space = joinNamespace(visitor.name_space, sub_namespace);
  declare_config(config);
  visitor.name_space = name_space_before;
  visitor.data.name = name_before;
}

}  // namespace config::internal
