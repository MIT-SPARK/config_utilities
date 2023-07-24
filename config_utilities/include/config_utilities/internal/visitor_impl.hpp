#pragma once

#include <exception>
#include <iostream>
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
MetaData Visitor::setValues(ConfigT& config,
                            const YAML::Node& node,
                            const bool print_warnings,
                            const std::string& name_space,
                            const std::string& field_name_prefix) {
  Visitor visitor(Mode::kSet, name_space, field_name_prefix);
  visitor.parser.setNode(node);
  declare_config(config);
  visitor.extractErrors();

  if (print_warnings && visitor.data.hasErrors()) {
    Logger::logWarning(Formatter::formatErrors(visitor.data, "Errors parsing config", Severity::kWarning));
  }
  return visitor.data;
}

template <typename ConfigT>
MetaData Visitor::getValues(const ConfigT& config,
                            const bool print_warnings,
                            const std::string& name_space,
                            const std::string& field_name_prefix) {
  Visitor visitor(Mode::kGet, name_space, field_name_prefix);
  // NOTE: We know that in mode kGet, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));
  visitor.extractErrors();
  mergeYamlNodes(visitor.data.data, visitor.parser.getNode());

  if (Settings::instance().indicate_default_values) {
    flagDefaultValues<ConfigT>(visitor.data);
  }

  if (print_warnings && visitor.data.hasErrors()) {
    Logger::logWarning(Formatter::formatErrors(visitor.data, "Errors parsing config", Severity::kWarning));
  }
  return visitor.data;
}

template <typename ConfigT>
MetaData Visitor::getChecks(const ConfigT& config) {
  Visitor visitor(Mode::kCheck);
  // NOTE: We know that in mode kCheck, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));
  visitor.extractErrors();
  return visitor.data;
}

template <typename ConfigT>
MetaData Visitor::subVisit(ConfigT& config, const bool print_warnings, const std::string& field_name_prefix) {
  Visitor& current_visitor = Visitor::instance();
  switch (current_visitor.mode) {
    case Visitor::Mode::kGet:
      return getValues(config, print_warnings, current_visitor.name_space, field_name_prefix);
    case Visitor::Mode::kSet:
      return setValues(
          config, current_visitor.parser.getNode(), print_warnings, current_visitor.name_space, field_name_prefix);
    case Visitor::Mode::kCheck:
      return getChecks(config);
    default:

      return MetaData();
  }
}

template <typename T, typename std::enable_if<!isConfig<T>(), bool>::type = true>
void Visitor::visitField(T& field, const std::string& field_name, const std::string& unit) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode == Visitor::Mode::kSet) {
    visitor.parser.fromYaml(field_name, field, visitor.name_space, visitor.field_name_prefix);
  }

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetDefaults) {
    FieldInfo& info = visitor.data.field_infos.emplace_back();
    visitor.parser.toYaml(field_name, field, visitor.name_space, visitor.field_name_prefix);
    info.value = YamlParser::toYaml(field);
    info.name = field_name;
    info.unit = unit;
  }
}

template <typename EnumT>
void Visitor::visitEnumField(EnumT& field,
                             const std::string& field_name,
                             const std::map<EnumT, std::string>& enum_names) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode == Visitor::Mode::kCheck) {
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
    if (visitor.parser.fromYaml(field_name, place_holder, visitor.name_space, visitor.field_name_prefix)) {
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

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetDefaults) {
    FieldInfo& info = visitor.data.field_infos.emplace_back();
    const auto it = enum_names.find(field);
    std::string value;
    if (it == enum_names.end()) {
      visitor.data.errors.emplace_back("Value of enum field '" + field_name + "' is out of the defined range.");
      value = "<Invalid Enum Name>";
    } else {
      value = it->second;
    }
    visitor.parser.toYaml(field_name, value, visitor.name_space, visitor.field_name_prefix);
    info.value = YamlParser::toYaml(value);
    info.name = field_name;
  }
}

template <typename T>
void Visitor::visitCheck(Visitor::CheckMode mode, const T& param, const T& value, const std::string& name) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.checker.setFieldNamePrefix(visitor.field_name_prefix);

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
  visitor.checker.setFieldNamePrefix(visitor.field_name_prefix);
  visitor.checker.checkInRange(param, lower, upper, name);
}

template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type = true>
void Visitor::visitField(ConfigT& config, const std::string& field_name, const std::string& /* unit */) {
  // Visits a subconfig field.
  Visitor& visitor = Visitor::instance();
  if (visitor.mode == Visitor::Mode::kGetDefaults) {
    return;
  }
  MetaData& data = visitor.data;

  // Add the field info.
  FieldInfo& info = data.field_infos.emplace_back();
  info.name = field_name;
  info.subconfig_id = data.sub_configs.size();
  data.current_field_name = field_name;

  // Visit subconfig.
  const std::string new_prefix =
      visitor.field_name_prefix + (Settings::instance().index_subconfig_field_names ? field_name + "." : "");
  MetaData& new_data = data.sub_configs.emplace_back(Visitor::subVisit(config, false, new_prefix));

  // Aggregate data.
  if (visitor.mode == Visitor::Mode::kGet) {
    // When getting data add the new data also to the parent data node. This automatically using the correct namespace.
    mergeYamlNodes(data.data, new_data.data);
    if (Settings::instance().indicate_default_values) {
      info.is_default = true;
      for (const auto& sub_info : new_data.field_infos) {
        if (!sub_info.is_default) {
          info.is_default = false;
          break;
        }
      }
    }
  }
}

template <typename ConfigT>
void Visitor::visitBase(ConfigT& config) {
  // For now simply pass the call to the base config, treating it as a single config object.
  // NOTE(lschmid): Alternatives are to treat it as a subconfig for clearer readability.
  Visitor& visitor = Visitor::instance();

  // Make sure each object is only visited once. The typeinfo and address of object should be a repeatable and unique
  // identifier.
  std::stringstream ss;
  ss << typeid(ConfigT).name() << static_cast<ConfigT*>(&config);
  const std::string key = ss.str();
  if (visitor.visited_base_configs.find(key) != visitor.visited_base_configs.end()) {
    return;
  }
  visitor.visited_base_configs.insert(key);

  // Call declare_config of the base while making sure the namespaces are persistent afterwards.
  const bool config_needs_name = visitor.data.name.empty();
  OpenNameSpace::performOperationWithGuardedNs(visitor.open_namespaces, [&config]() { declare_config(config); });

  // If the config name has changed reset it to avoid name conflicts. Configs should always be named after the most
  // derived config.
  if (config_needs_name) {
    visitor.data.name = "";
  }
}

template <typename ConfigT>
void Visitor::flagDefaultValues(MetaData& data) {
  // Get defaults from a default constructed ConfigT. Extract the default values of all non-config fields. Subconfigs
  // are managed separately.
  ConfigT default_config;
  Visitor visitor(Mode::kGetDefaults);
  declare_config(default_config);
  const MetaData& default_data = visitor.data;

  // Compare all fields. These should always be in the same order if they are from the same config, but exclude
  // subconfigs.
  size_t i = 0;
  for (FieldInfo& info : data.field_infos) {
    if (info.subconfig_id >= 0) {
      continue;
    }
   // Corresponding field info should always exist
   const auto& default_info = default_data.field_infos.at(i);
   ++i;
    // NOTE(lschmid): Operator YAML::Node== checks for identity, not equality. Since these are all scalars, comparing
    // the formatted strings should be identical.
    if (internal::dataToString(info.value) == internal::dataToString(default_info.value)) {
      info.is_default = true;
    }
  }
}

}  // namespace config::internal
