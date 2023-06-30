#pragma once

#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "config_utilities/internal/validity_checker.h"

namespace config::internal {

/**
 * @brief Meta-information struct that interfaces all the communication data when interacting with the configs. YAML is
 * used as internal data representation.
 */
struct MetaData {
  // We always get the name of the config if possible.
  std::string name = "Unnamed Config";

  // Yaml node used to get or set the data of a config.
  YAML::Node data;

  // All units where specified. units[field_name] = unit
  std::map<std::string, std::string> units;

  // All warnings issued by the validity checker.
  std::vector<std::string> warnings;
};

enum class CheckMode { kGT, kGE, kLT, kLE, kEQ, kNE };

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
  static void setValues(ConfigT& config, const YAML::Node& node) {
    Visitor visitor = Visitor::create();
    visitor.data.data = node;
    visitor.mode = Visitor::Mode::kSet;
    declare_config(config);
  }

  template <typename ConfigT>
  static MetaData getValues(const ConfigT& config) {
    Visitor visitor = Visitor::create();
    visitor.mode = Visitor::Mode::kGet;
    // NOTE: We know that in mode kGet, the config is not modified.
    declare_config(const_cast<ConfigT&>(config));
    return visitor.data;
  }

  template <typename ConfigT>
  static MetaData getChecks(const ConfigT& config) {
    Visitor visitor = Visitor::create();
    visitor.mode = Visitor::Mode::kCheck;
    // NOTE: We know that in mode kCheck, the config is not modified.
    declare_config(const_cast<ConfigT&>(config));
    visitor.data.warnings = visitor.validity_checker.getWarnings();
    return visitor.data;
  }

 private:
  // Forward declare access to the visiting functions.
  friend void visitName(const std::string&);
  template <typename T>
  friend void visitField(T&, const std::string&, const std::string&);
  template <typename T>
  friend void visitCheck(CheckMode, const T&, const T&, const std::string&);
  template <typename T>
  friend void visitCheckInRange(const T&, const T&, const T&, const std::string&);
  friend void visitCheckCondition(bool, const std::string&);

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
  ValidityChecker validity_checker;

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
    // TODO(lschmid): Implement.
  } else if (visitor.mode == Visitor::Mode::kGet) {
    // TODO(lschmid): Double check yaml parsing here.
    visitor.data.data[field_name] = field;
    if (!unit.empty()) {
      visitor.data.units[field_name] = unit;
    }
  }
}

template <typename T>
void visitCheck(CheckMode mode, const T& param, const T& value, const std::string& name) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }

  switch (mode) {
    case CheckMode::kGT:
      visitor.validity_checker.checkGT(param, value, name);
      return;
    case CheckMode::kGE:
      visitor.validity_checker.checkGE(param, value, name);
      return;
    case CheckMode::kLT:
      visitor.validity_checker.checkLT(param, value, name);
      return;
    case CheckMode::kLE:
      visitor.validity_checker.checkLE(param, value, name);
      return;
    case CheckMode::kEQ:
      visitor.validity_checker.checkEq(param, value, name);
      return;
    case CheckMode::kNE:
      visitor.validity_checker.checkNE(param, value, name);
      return;
  }
}

template <typename T>
void visitCheckInRange(const T& param, const T& lower, const T& upper, const std::string& name) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.validity_checker.checkInRange(param, lower, upper, name);
}

void visitCheckCondition(bool condition, const std::string& error_message) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.validity_checker.checkCondition(condition, error_message);
}

}  // namespace config::internal
