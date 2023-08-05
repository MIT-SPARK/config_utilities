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
                            const std::string& field_name) {
  Visitor visitor(Mode::kSet, name_space, field_name);
  visitor.data.data.reset(node);
  declare_config(config);
  if (print_warnings && visitor.data.hasErrors()) {
    Logger::logWarning(Formatter::formatErrors(visitor.data, "Errors parsing config", Severity::kWarning));
  }
  return visitor.data;
}

template <typename ConfigT>
MetaData Visitor::getValues(const ConfigT& config,
                            const bool print_warnings,
                            const std::string& name_space,
                            const std::string& field_name) {
  Visitor visitor(Mode::kGet, name_space, field_name);
  // NOTE: We know that in mode kGet, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));
  if (Settings::instance().indicate_default_values) {
    flagDefaultValues(config, visitor.data);
  }
  if (print_warnings && visitor.data.hasErrors()) {
    Logger::logWarning(Formatter::formatErrors(visitor.data, "Errors parsing config", Severity::kWarning));
  }
  return visitor.data;
}

template <typename ConfigT>
MetaData Visitor::getChecks(const ConfigT& config, const std::string& field_name) {
  Visitor visitor(Mode::kCheck, "", field_name);
  // NOTE: We know that in mode kCheck, the config is not modified.
  declare_config(const_cast<ConfigT&>(config));
  return visitor.data;
}

template <typename ConfigT>
MetaData Visitor::subVisit(ConfigT& config,
                           const bool print_warnings,
                           const std::string& field_name,
                           bool name_is_namespace,
                           const std::string& field_ns) {
  if (name_is_namespace) {
    enter_namespace(field_ns.empty() ? field_name : field_ns);
  }

  MetaData data;
  Visitor& current_visitor = Visitor::instance();
  switch (current_visitor.mode) {
    case Visitor::Mode::kGet:
      data = getValues(config, print_warnings, current_visitor.name_space, field_name);
      break;
    case Visitor::Mode::kSet:
      data = setValues(config, current_visitor.data.data, print_warnings, current_visitor.name_space, field_name);
      break;
    case Visitor::Mode::kCheck:
      data = getChecks(config, field_name);
      break;
    default:
      break;
  }

  if (name_is_namespace) {
    exit_namespace();
  }
  return data;
}

template <typename T, typename std::enable_if<!isConfig<T>(), bool>::type = true>
void Visitor::visitField(T& field, const std::string& field_name, const std::string& unit) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode == Visitor::Mode::kSet) {
    std::string error;
    YamlParser::fromYaml(visitor.data.data, field_name, field, visitor.name_space, error);
    if (!error.empty()) {
      visitor.data.errors.emplace_back(new Warning(field_name, error));
    }
  }

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetDefaults) {
    FieldInfo& info = visitor.data.field_infos.emplace_back();
    std::string error;
    YAML::Node node = YamlParser::toYaml(field_name, field, visitor.name_space, error);
    mergeYamlNodes(visitor.data.data, node);
    // This stores a reference to the node in the data.
    info.value = lookupNamespace(node, joinNamespace(visitor.name_space, field_name));
    info.name = field_name;
    info.unit = unit;
    if (!error.empty()) {
      visitor.data.errors.emplace_back(new Warning(field_name, error));
    }
  }
}

template <typename Conversion, typename T, typename std::enable_if<!isConfig<T>(), bool>::type = true>
void Visitor::visitField(T& field, const std::string& field_name, const std::string& unit) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode == Visitor::Mode::kSet) {
    std::string error;
    auto intermediate = Conversion::toIntermediate(field, error);
    error.clear();  // We don't care about setting up the intermediate just to get data.

    YamlParser::fromYaml(visitor.data.data, field_name, intermediate, visitor.name_space, error);
    if (!error.empty()) {
      visitor.data.errors.emplace_back(new Warning(field_name, error));
      error.clear();
    }

    Conversion::fromIntermediate(intermediate, field, error);
    if (!error.empty()) {
      visitor.data.errors.emplace_back(new Warning(field_name, error));
    }
  }

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetDefaults) {
    FieldInfo& info = visitor.data.field_infos.emplace_back();
    std::string error;
    const auto intermediate = Conversion::toIntermediate(field, error);
    if (!error.empty()) {
      visitor.data.errors.emplace_back(new Warning(field_name, error));
      error.clear();
    }
    YAML::Node node = YamlParser::toYaml(field_name, intermediate, visitor.name_space, error);
    mergeYamlNodes(visitor.data.data, node);
    // This stores a reference to the node in the data.
    info.value = lookupNamespace(node, joinNamespace(visitor.name_space, field_name));
    info.name = field_name;
    info.unit = unit;
    if (!error.empty()) {
      visitor.data.errors.emplace_back(new Warning(field_name, error));
    }
  }
}

// Visits a subconfig field.
template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type = true>
void Visitor::visitField(ConfigT& config,
                         const std::string& field_name,
                         bool name_is_namespace,
                         const std::string& field_ns) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode == Visitor::Mode::kGetDefaults) {
    return;
  }

  // Visit subconfig.
  MetaData& data = visitor.data;
  MetaData& new_data =
      data.sub_configs.emplace_back(Visitor::subVisit(config, false, field_name, name_is_namespace, field_ns));

  // Aggregate data.
  if (visitor.mode == Visitor::Mode::kGet) {
    // When getting data add the new data also to the parent data node. This automatically using the correct namespace.
    mergeYamlNodes(data.data, new_data.data);
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

template <typename ConfigT, typename std::enable_if<!is_virtual_config<ConfigT>::value, bool>::type = true>
MetaData Visitor::getDefaults(const ConfigT& config) {
  Visitor visitor(Mode::kGetDefaults);
  ConfigT default_config;
  declare_config(default_config);
  return visitor.data;
}

template <typename ConfigT, typename std::enable_if<is_virtual_config<ConfigT>::value, bool>::type = true>
MetaData Visitor::getDefaults(const ConfigT& config) {
  Visitor visitor(Mode::kGetDefaults);
  if (config.isSet()) {
    auto default_config = config.config_->createDefault();
    default_config->onDeclareConfig();
  }
  return visitor.data;
}

template <typename ConfigT>
void Visitor::flagDefaultValues(const ConfigT& config, MetaData& data) {
  // Get defaults from a default constructed ConfigT. Extract the default values of all non-config fields. Subconfigs
  // are managed separately.
  const MetaData default_data = getDefaults(config);

  // Compare all fields. These should always be in the same order if they are from the same config and exclude
  // subconfigs.
  size_t default_idx = 0;
  for (size_t i = 0; i < data.field_infos.size(); ++i) {
    FieldInfo& info = data.field_infos.at(i);
    // note that default config may not contain the same fields as the current config
    // if certain fields are conditionally enabled
    for (; default_idx < default_data.field_infos.size(); ++default_idx) {
      if (default_data.field_infos.at(default_idx).name == info.name) {
        break;
      }
    }

    if (default_idx >= default_data.field_infos.size()) {
      break;
    }

    // NOTE(lschmid): Operator YAML::Node== checks for identity, not equality. Since these are all scalars, comparing
    // the formatted strings should be identical.
    const auto& default_info = default_data.field_infos.at(default_idx);
    if (internal::dataToString(info.value) == internal::dataToString(default_info.value)) {
      info.is_default = true;
    }
  }
}

}  // namespace config::internal
