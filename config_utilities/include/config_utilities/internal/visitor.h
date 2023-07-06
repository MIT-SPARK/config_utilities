#pragma once

#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/meta_data.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/validity_checker.h"
#include "config_utilities/internal/yaml_parser.h"
#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/traits.h"

namespace config::internal {

/**
 * @brief The visitor gets and sets information between the meta-data and configs. It is hidden via in-thread singleton
 * sequential access to make the public user interface cleaner. This is thread safe and creates one visitor instance per
 * thread, so it can be used as if single-threaded.
 */
struct Visitor {
  ~Visitor() {
    std::lock_guard<std::mutex> lock(instance_mutex);
    instances.erase(id);
  }

  // Interfaces for all internal tools interact with configs through the visitor.
  template <typename ConfigT>
  static MetaData setValues(ConfigT& config, const YAML::Node& node, bool print_warnings = true) {
    Visitor visitor = Visitor::create();
    visitor.parser.node() = node;
    visitor.mode = Visitor::Mode::kSet;
    declare_config(config);
    visitor.data.errors.insert(
        visitor.data.errors.end(), visitor.parser.errors().begin(), visitor.parser.errors().end());
    if (print_warnings && visitor.data.hasErrors()) {
      Logger::logWarning(Formatter::formatErrors(visitor.data, "Errors parsing config", Formatter::Severity::kWarning));
    }
    return visitor.data;
  }

  template <typename ConfigT>
  static MetaData getValues(const ConfigT& config, bool print_warnings = true) {
    Visitor visitor = Visitor::create();
    visitor.mode = Visitor::Mode::kGet;
    // NOTE: We know that in mode kGet, the config is not modified.
    declare_config(const_cast<ConfigT&>(config));
    visitor.data.errors.insert(
        visitor.data.errors.end(), visitor.parser.errors().begin(), visitor.parser.errors().end());
    visitor.data.data = visitor.parser.node();
    if (print_warnings && visitor.data.hasErrors()) {
      Logger::logWarning(Formatter::formatErrors(visitor.data, "Errors parsing config", Formatter::Severity::kWarning));
    }
    return visitor.data;
  }

  template <typename ConfigT>
  static MetaData getChecks(const ConfigT& config) {
    Visitor visitor = Visitor::create();
    visitor.mode = Visitor::Mode::kCheck;
    // NOTE: We know that in mode kCheck, the config is not modified.
    declare_config(const_cast<ConfigT&>(config));
    visitor.data.errors = visitor.checker.getWarnings();
    return visitor.data;
  }

 private:
  enum class CheckMode { kGT, kGE, kLT, kLE, kEQ, kNE };

  // Forward declare access to the visiting functions.
  friend void visitName(const std::string&);
  template <typename T>
  friend void visitField(T&, const std::string&, const std::string&);
  template <typename EnumT>
  friend void visitEnumField(EnumT&, const std::string&, const std::map<EnumT, std::string>&);
  template <typename T>
  friend void visitCheck(Visitor::CheckMode, const T&, const T&, const std::string&);
  template <typename T>
  friend void visitCheckInRange(const T&, const T&, const T&, const std::string&);
  friend void visitCheckCondition(bool, const std::string&);
  template <typename ConfigT>
  friend void visitSubconfig(ConfigT&, const std::string&, const std::string&);

  // Create and access the meta data for the current thread. Lifetime of the meta data is managed internally by the
  // objects. Note that meta data always needs to be created before it can be accessed. In short, 'instance()' is only
  // to be used within the 'declare_config()' function, whereas 'create()' is to be used to extract data from a struct
  // by calling 'declare_config()'.
  static Visitor create() {
    const std::thread::id id = std::this_thread::get_id();
    return Visitor(id);
  }

  static Visitor& instance() {
    std::lock_guard<std::mutex> lock(instance_mutex);
    const std::thread::id id = std::this_thread::get_id();
    auto it = instances.find(id);
    if (it == instances.end()) {
      // This should never happen as meta data are managed internally. Caught here for debugging.
      std::stringstream ss;
      ss << "Visitor for thread " << id << " accessed but was not created.";
      throw std::runtime_error(ss.str());
    }
    return *instances.at(id);
  }

  // Create one instance per thread and store the reference to it.
  explicit Visitor(std::thread::id _id) : id(_id) {
    std::lock_guard<std::mutex> lock(instance_mutex);
    if (instances.find(id) != instances.end()) {
      // This should never happen as  meta data are managed internally. Caught here for debugging.
      std::stringstream ss;
      ss << "Tried to create Visitor for thread " << id << " which already exists.";
      throw std::runtime_error(ss.str());
    }
    instances[id] = this;
  }

  // Which operations to perform on the data.
  enum class Mode { kUnspecified, kGet, kSet, kCheck } mode = Mode::kUnspecified;

  // Messenger data.
  MetaData data;

  // Internal data to handle visits.
  ValidityChecker checker;
  YamlParser parser;
  std::string name_prefix = "";
  std::string name_space = "";

  // Static registration to get access to the correct instance.
  inline static std::map<std::thread::id, Visitor*> instances;
  inline static std::mutex instance_mutex;

  // Member data.
  const std::thread::id id;
};

// Implementation of visits of the fields exposed in config.h
void visitName(const std::string& name) { Visitor::instance().data.name = name; }

template <typename T>
void visitField(T& field, const std::string& field_name, const std::string& unit) {
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
void visitEnumField(EnumT& field, const std::string& field_name, const std::map<EnumT, std::string>& enum_names) {
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
void visitCheck(Visitor::CheckMode mode, const T& param, const T& value, const std::string& name) {
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
void visitCheckInRange(const T& param, const T& lower, const T& upper, const std::string& name) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.checker.checkInRange(param, lower, upper, name);
}

void visitCheckCondition(bool condition, const std::string& error_message) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.checker.checkCondition(condition, error_message);
}

template <typename ConfigT>
void visitSubconfig(ConfigT& config, const std::string& field_name, const std::string& sub_namespace) {
  Visitor& visitor = Visitor::instance();

  if (visitor.mode != Visitor::Mode::kSet && visitor.mode != Visitor::Mode::kGet &&
      visitor.mode != Visitor::Mode::kCheck) {
    return;
  }

  // Check is a configT.
  if (!isConfig<ConfigT>()) {
    visitor.data.errors.emplace_back("Subconfig field '" + field_name + "' has not beend declared a config.");
    return;
  }

  // Store state as was before.
  const MetaData data_before = visitor.data;
  const std::string name_space_before = visitor.name_space;
  const std::string name_prefix_before = visitor.name_prefix;

  // Set new data.
  // TODO(lschmid): This could also be part of the formatter but a bit complicated for now.
  constexpr bool index_errors_with_subconfig_names = true;
  if (index_errors_with_subconfig_names) {
    visitor.name_prefix += field_name + ".";
  }
  visitor.name_space = joinNamespace(visitor.name_space, sub_namespace);
  visitor.data = MetaData();

  // Visit subconfig.
  declare_config(config);

  // Aggregate data.
  MetaData data_after = visitor.data;
  if (visitor.mode == Visitor::Mode::kGet) {
    data_after.data = lookupNamespace(YAML::Clone(visitor.parser.node()), visitor.name_space);
  }

  // Restore state.
  visitor.name_space = name_space_before;
  visitor.name_prefix = name_prefix_before;
  visitor.data = data_before;
  visitor.data.sub_configs.emplace_back(std::move(data_after));
}

}  // namespace config::internal
