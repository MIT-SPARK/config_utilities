/** -----------------------------------------------------------------------------
 * Copyright (c) 2023 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
 * AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2023
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#pragma once

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
MetaData Visitor::setValues(ConfigT& config,
                            const YAML::Node& node,
                            const bool print_warnings,
                            const std::string& name_space,
                            const std::string& field_name,
                            const bool print_missing) {
  Visitor visitor(Mode::kSet, name_space, field_name);
  visitor.meta_data.data.reset(node);
  ::config::declare_config(config);
  if (print_warnings && visitor.meta_data.hasErrors()) {
    Logger::logWarning(Formatter::formatErrors(visitor.meta_data, "Errors parsing config", Severity::kWarning));
  }

  if (print_missing && Settings::instance().printing.show_missing && visitor.meta_data.hasMissing()) {
    Logger::logWarning(Formatter::formatMissing(visitor.meta_data, "Missing fields from config", Severity::kWarning));
  }

  return visitor.meta_data;
}

template <typename ConfigT>
MetaData Visitor::getValues(const ConfigT& config,
                            const bool print_warnings,
                            const std::string& name_space,
                            const std::string& field_name) {
  Visitor visitor(Mode::kGet, name_space, field_name);
  // NOTE(lschmid): We know that in mode kGet, the config is not modified.
  ::config::declare_config(const_cast<ConfigT&>(config));

  if (Settings::instance().printing.show_defaults) {
    Visitor::getDefaultValues(config, visitor.meta_data);
  }
  if (print_warnings && visitor.meta_data.hasErrors()) {
    Logger::logWarning(Formatter::formatErrors(visitor.meta_data, "Errors parsing config", Severity::kWarning));
  }

  return visitor.meta_data;
}

template <typename ConfigT>
MetaData Visitor::getInfo(const ConfigT& config, const std::string& name_space, const std::string& field_name) {
  Visitor visitor(Mode::kGetInfo, name_space, field_name);
  // NOTE(lschmid): We know that in mode kGetInfo, the config is not modified.
  ::config::declare_config(const_cast<ConfigT&>(config));
  Visitor::getDefaultValues(config, visitor.meta_data);

  // Try to associate check data with the fields by name.
  visitor.meta_data.performOnAll([](MetaData& data) {
    for (const auto& check : data.checks) {
      for (auto& field_info : data.field_infos) {
        if (field_info.name == check->name()) {
          field_info.input_info = FieldInputInfo::merge(check->fieldInputInfo(), field_info.input_info);
          break;
        }
      }
    }
  });
  return visitor.meta_data;
}

template <typename ConfigT>
MetaData Visitor::getChecks(const ConfigT& config, const std::string& field_name) {
  Visitor visitor(Mode::kCheck, "", field_name);
  // NOTE(lschmid): We know that in mode kCheck, the config is not modified.
  ::config::declare_config(const_cast<ConfigT&>(config));
  return visitor.meta_data;
}

template <typename ConfigT>
MetaData Visitor::subVisit(ConfigT& config,
                           const bool print_warnings,
                           const std::string& field_name,
                           const std::string& sub_ns) {
  MetaData data;
  Visitor& current_visitor = Visitor::instance();
  const std::string name_space = joinNamespace(current_visitor.name_space, sub_ns);
  switch (current_visitor.mode) {
    case Visitor::Mode::kGet:
      data = getValues(config, print_warnings, name_space, field_name);
      break;
    case Visitor::Mode::kSet:
      data = setValues(config, current_visitor.meta_data.data, print_warnings, name_space, field_name, false);
      break;
    case Visitor::Mode::kCheck:
      data = getChecks(config, field_name);
      break;
    case Visitor::Mode::kGetInfo:
      data = getInfo(config, name_space, field_name);
    default:
      break;
  }
  return data;
}

// Visit a non-config field.
template <typename T, typename std::enable_if<!isConfig<T>(), bool>::type>
void Visitor::visitField(T& field, const std::string& field_name, const std::string& unit) {
  auto& visitor = Visitor::instance();

  // record the field that we visited without storing the value
  // kGet will populate the value field
  auto& info = visitor.meta_data.field_infos.emplace_back();
  info.name = field_name;
  info.unit = unit;
  info.ns = visitor.additionalNamespace();

  if (visitor.mode == Visitor::Mode::kSet) {
    std::string error;
    // This is the input value, stores a reference to the node in the data.
    info.value = lookupNamespace(visitor.meta_data.data, joinNamespace(visitor.name_space, field_name));
    const auto parsed = YamlParser::fromYaml<T>(info.value, &error);
    if (parsed) {
      field = *parsed;
    }
    info.was_parsed = static_cast<bool>(parsed);
    if (!error.empty()) {
      visitor.meta_data.errors.emplace_back(new Warning(field_name, error));
    }
  }

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetInfo) {
    std::string error;
    // Aggregate the YAML data in the global data.
    auto node = YamlParser::toYaml(field, &error);
    const std::string ns = joinNamespace(visitor.name_space, field_name);
    moveDownNamespace(node, ns);
    mergeYamlNodes(visitor.meta_data.data, node);
    // Stores a reference to the node in the data.
    info.value = lookupNamespace(visitor.meta_data.data, ns);
    if (!error.empty()) {
      visitor.meta_data.errors.emplace_back(new Warning(field_name, error));
    }

    // Get type information if requested.
    if (visitor.mode == Visitor::Mode::kGetInfo) {
      auto input_info = createFieldInputInfo<T>();
      info.input_info = FieldInputInfo::merge(input_info, info.input_info);
    }
  }
}

// Visits a non-config field with conversion.
template <typename Conversion, typename T>
void Visitor::visitField(T& field, const std::string& field_name, const std::string& unit) {
  auto& visitor = Visitor::instance();

  if (visitor.mode == Visitor::Mode::kSet) {
    std::string error;
    auto intermediate = Conversion::toIntermediate(field, error);
    error.clear();  // We don't care about setting up the intermediate just to get data.

    Visitor::visitField(intermediate, field_name, unit);

    Conversion::fromIntermediate(intermediate, field, error);
    if (!error.empty()) {
      visitor.meta_data.errors.emplace_back(new Warning(field_name, error));
    }
  }

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetInfo) {
    std::string error;
    auto intermediate = Conversion::toIntermediate(field, error);
    if (!error.empty()) {
      visitor.meta_data.errors.emplace_back(new Warning(field_name, error));
    }

    Visitor::visitField(intermediate, field_name, unit);

    // Get type information if requested.
    if (visitor.mode == Visitor::Mode::kGetInfo) {
      Visitor::getFieldInputInfo<Conversion, T>(intermediate, field_name);
    }
  }
}

// Visits a single subconfig field.
template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type>
void Visitor::visitField(ConfigT& config, const std::string& field_name, const std::string& name_space) {
  Visitor& visitor = Visitor::instance();

  // Visit subconfig.
  MetaData& data = visitor.meta_data;
  MetaData& new_data = data.sub_configs.emplace_back(Visitor::subVisit(config, false, field_name, name_space));

  // Aggregate data.
  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetInfo) {
    // When getting data add the new data also to the parent data node. This is automatically using the correct
    // namespace.
    mergeYamlNodes(data.data, new_data.data);
  }
}

// Visit a vector of subconfigs.
template <typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type>
void Visitor::visitField(std::vector<ConfigT>& config, const std::string& field_name, const std::string& /* unit */) {
  Visitor& visitor = Visitor::instance();

  if (visitor.mode == Visitor::Mode::kSet) {
    const auto array_ns = joinNamespace(visitor.name_space, field_name);
    const auto subnode = lookupNamespace(visitor.meta_data.data, array_ns);
    if (!subnode) {
      // don't override the field if not present.
      return;
    }

    // When setting the values first allocate the correct amount of configs.
    config.clear();
    const std::vector<YAML::Node> nodes = getNodeArray(subnode);
    size_t index = 0;
    for (const auto& node : nodes) {
      ConfigT& sub_config = config.emplace_back();
      auto& new_data =
          visitor.meta_data.sub_configs.emplace_back(setValues(sub_config, node, false, "", field_name, false));
      new_data.array_config_index = index++;
    }
  }

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetInfo) {
    YAML::Node array_node(YAML::NodeType::Sequence);
    size_t index = 0;
    const std::string name_space = joinNamespace(visitor.name_space, field_name);
    for (const auto& sub_config : config) {
      if (visitor.mode == Visitor::Mode::kGetInfo) {
        visitor.meta_data.sub_configs.emplace_back(getInfo(sub_config, name_space, field_name));
      } else {
        visitor.meta_data.sub_configs.emplace_back(getValues(sub_config, false, name_space, field_name));
      }
      MetaData& new_data = visitor.meta_data.sub_configs.back();
      array_node.push_back(YAML::Clone(lookupNamespace(new_data.data, name_space)));
      new_data.array_config_index = index++;
    }

    // TODO(lschmid): Add info for empty vectors for getInfo.
    moveDownNamespace(array_node, name_space);
    mergeYamlNodes(visitor.meta_data.data, array_node);
  }

  if (visitor.mode == Visitor::Mode::kCheck) {
    size_t index = 0;
    for (const auto& sub_config : config) {
      auto& new_data = visitor.meta_data.sub_configs.emplace_back(getChecks(sub_config, field_name));
      new_data.array_config_index = index++;
    }
  }
}

// Visit a map of subconfigs.
template <typename K, typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type>
void Visitor::visitField(std::map<K, ConfigT>& config, const std::string& field_name, const std::string& unit) {
  Visitor& visitor = Visitor::instance();

  // copy current config state
  OrderedMap<K, ConfigT> intermediate(config.begin(), config.end());

  // make use of more general named parsing
  visitField<K, ConfigT>(intermediate, field_name, unit);

  // assign parsed configs to actual map if we're setting the config
  if (visitor.mode == Visitor::Mode::kSet) {
    config.clear();
    // note that we don't use insert here to guarantee that duplicates behave as expected (overriding with the last)
    for (const auto& [key, value] : intermediate) {
      config[key] = value;
    }
  }
}

template <typename K, typename ConfigT, typename std::enable_if<isConfig<ConfigT>(), bool>::type>
void Visitor::visitField(OrderedMap<K, ConfigT>& config, const std::string& field_name, const std::string& /* unit */) {
  Visitor& visitor = Visitor::instance();

  if (visitor.mode == Visitor::Mode::kSet) {
    const auto map_ns = joinNamespace(visitor.name_space, field_name);
    const auto subnode = lookupNamespace(visitor.meta_data.data, map_ns);
    if (!subnode) {
      return;  // don't override the field if not present
    }

    // When setting the values first allocate the correct amount of configs.
    config.clear();
    const auto nodes = getNodeMap(subnode);
    for (auto&& [key, node] : nodes) {
      auto& entry = config.emplace_back();
      entry.first = key.template as<K>();
      auto& new_data =
          visitor.meta_data.sub_configs.emplace_back(setValues(entry.second, node, false, "", field_name, false));
      new_data.map_config_key = key.template as<std::string>();
    }
  }

  if (visitor.mode == Visitor::Mode::kGet || visitor.mode == Visitor::Mode::kGetInfo) {
    const std::string name_space = joinNamespace(visitor.name_space, field_name);
    YAML::Node map_node(YAML::NodeType::Map);
    for (auto&& [key, sub_config] : config) {
      if (visitor.mode == Visitor::Mode::kGetInfo) {
        visitor.meta_data.sub_configs.emplace_back(getInfo(sub_config, name_space, field_name));
      } else {
        visitor.meta_data.sub_configs.emplace_back(getValues(sub_config, false, name_space, field_name));
      }
      MetaData& new_data = visitor.meta_data.sub_configs.back();
      map_node[key] = YAML::Clone(lookupNamespace(new_data.data, name_space));
      new_data.map_config_key = YAML::Node(key).as<std::string>();
    }

    if (visitor.mode == Visitor::Mode::kGetInfo && config.empty()) {
      // When getting info for empty maps still show them.
      // TODO(lschmid): Implement, currently empty maps will not show up in the info.
    }
    moveDownNamespace(map_node, name_space);
    mergeYamlNodes(visitor.meta_data.data, map_node);
  }

  if (visitor.mode == Visitor::Mode::kCheck) {
    for (auto&& [key, sub_config] : config) {
      visitor.meta_data.sub_configs.emplace_back(getChecks(sub_config, field_name));
      visitor.meta_data.sub_configs.back().map_config_key = YAML::Node(key).as<std::string>();
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
  const bool config_needs_name = visitor.meta_data.name.empty();
  OpenNameSpace::performOperationWithGuardedNs(visitor.open_namespaces,
                                               [&config]() { ::config::declare_config(config); });

  // If the config name has changed reset it to avoid name conflicts. Configs should always be named after the most
  // derived config.
  if (config_needs_name) {
    visitor.meta_data.name = "";
  }
}

template <typename ConfigT, typename std::enable_if<!is_virtual_config<ConfigT>::value, bool>::type>
MetaData Visitor::getDefaults(const ConfigT& /* config */) {
  Visitor visitor(Mode::kGet);
  ConfigT default_config;
  ::config::declare_config(default_config);
  return visitor.meta_data;
}

template <typename ConfigT, typename std::enable_if<is_virtual_config<ConfigT>::value, bool>::type>
MetaData Visitor::getDefaults(const ConfigT& config) {
  Visitor visitor(Mode::kGet);
  if (config.isSet()) {
    // Parse the type param.
    visitField(config.config_->type, Settings::instance().factory.type_param_name, "");

    // Parse a default initialized config.
    auto default_config = config.config_->createDefault();
    default_config->onDeclareConfig();
  }
  return visitor.meta_data;
}

template <typename ConfigT>
void Visitor::getDefaultValues(const ConfigT& config, MetaData& data) {
  // Get defaults from a default constructed ConfigT.
  const MetaData default_data = Visitor::getDefaults(config);

  // Compare all fields. These should always be in the same order if they are from the same config.
  for (auto& field_info : data.field_infos) {
    const auto match = default_data.findMatchingFieldInfo(field_info);
    if (match) {
      field_info.default_value = match->value;
    }
  }

  // getDefaultTypeParams(data, default_data);
  // Parse all immediate virtual subconfigs to get their default type params.
  for (auto& sub_data : data.sub_configs) {
    const auto match = default_data.findMatchingSubConfig(sub_data);
    if (sub_data.isVirtualConfig()) {
      // Find the type param. This should exist.
      for (auto& field_info : sub_data.field_infos) {
        if (field_info.name == Settings::instance().factory.type_param_name && field_info.is_meta_field) {
          if (!match) {
            field_info.default_value = YAML::Node();  // no default
          } else {
            const auto field_match = match->findMatchingFieldInfo(field_info);
            if (field_match) {
              field_info.default_value = field_match->value;
            } else {
              // NOTE(lschmid): This should never happen.
              field_info.default_value = YAML::Node();  // no default
            }
          }
        }
      }
    }
  }
}

// intentional no-op
template <typename Conversion,
          typename ConfigT,
          typename IntermediateT,
          typename std::enable_if<!hasFieldInputInfo<Conversion>() || isConfig<ConfigT>(), bool>::type>
void Visitor::getFieldInputInfo(const IntermediateT&, const std::string& field_name) {
  static_assert(!isConfig<ConfigT>() || !hasFieldInputInfo<Conversion>(),
                "Config types (with declare_config) cannot have field input information!");

  if (isConfig<ConfigT>()) {
    return;  // don't touch field info fields
  }

  auto& visitor = Visitor::instance();
  if (visitor.meta_data.field_infos.empty()) {
    visitor.meta_data.errors.emplace_back(
        new Warning(field_name, "Invalid parsing state! Field info should already exist!"));
  } else {
    // Default: Create the field input info of the intermediate type.
    auto input_info = createFieldInputInfo<IntermediateT>();
    auto& info = visitor.meta_data.field_infos.back();
    info.input_info = FieldInputInfo::merge(input_info, info.input_info);
  }
}

template <typename Conversion,
          typename ConfigT,
          typename IntermediateT,
          typename std::enable_if<hasFieldInputInfo<Conversion>() && !isConfig<ConfigT>(), bool>::type>
void Visitor::getFieldInputInfo(const IntermediateT&, const std::string& field_name) {
  auto input_info = Conversion::getFieldInputInfo();

  auto& visitor = Visitor::instance();
  if (visitor.meta_data.field_infos.empty()) {
    visitor.meta_data.errors.emplace_back(
        new Warning(field_name, "Invalid parsing state! Field info should already exist!"));
  } else {
    auto& info = visitor.meta_data.field_infos.back();
    info.input_info = FieldInputInfo::merge(input_info, info.input_info);
  }
}

}  // namespace config::internal
