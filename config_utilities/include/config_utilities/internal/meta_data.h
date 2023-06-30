#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include <glog/logging.h>

#include "config_utilities/internal/validity_checker.h"

namespace config::internal {

/**
 * @brief Meta-information struct to store data when interacting with the configs. This is thread safe and creates one
 * data instance per thread, so it can be used as if single-threaded.
 */
struct MetaData {
  // Create and access the meta data for the current thread. Lifetime of the meta data is managed internally by the
  // objects. Note that meta data always needs to be created before it can be accessed. In short, 'instance()' is only
  // to be used within the 'declare_config()' function, whereas 'create()' is to be used to extract data from a struct
  // by calling 'declare_config()'.
  static MetaData create() {
    const std::thread::id id = std::this_thread::get_id();
    return MetaData(id);
  }

  static MetaData& instance() {
    std::lock_guard<std::mutex> lock(instance_mutex);
    const std::thread::id id = std::this_thread::get_id();
    auto it = instances.find(id);
    if (it == instances.end()) {
      // This should never happen as operators are managed internally. Caught here for debugging.
      LOG(FATAL) << "MetaData for thread " << id << " accessed but was not created.";
    }
    return *instances.at(id);
  }

  ~MetaData() {
    std::lock_guard<std::mutex> lock(instance_mutex);
    instances.erase(id);
  };

  // Properties used for interaction with configs.
  // General.
  enum Mode { kUnset, kToString, kCheckValid, kVisit } mode = Mode::kUnset;
  std::string name = "Unnamed Config";

  // Validity Checks.
  ValidityChecker validity_checker;

  // Allow any visitor to access config data.
  template <typename T>
  void visit(T& param, const std::string& name, const std::string& unit) {}

 private:
  // Create one instance per thread and store the reference to it.
  explicit MetaData(std::thread::id _id) : id(_id) {
    std::lock_guard<std::mutex> lock(instance_mutex);
    if (instances.find(id) != instances.end()) {
      // This should never happen as operators are managed internally. Caught here for debugging.
      LOG(FATAL) << "Tried to create MetaData for thread " << id << " which already exists.";
    }
    instances[id] = this;
  }

  // Static registration to get access to the correct instance.
  inline static std::map<std::thread::id, MetaData*> instances;
  inline static std::mutex instance_mutex;

  // Member data.
  const std::thread::id id;
};

}  // namespace config::internal
