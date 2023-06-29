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
 * */
struct MetaData {
  // Create and access the meta data for the current thread. Lifetime of the meta data is managed internally by the
  // objects. Note that meta data always needs to be created before it can be accessed. In short, 'instance()' is only
  // to be used within the 'declare_config()' function, whereas 'create()' is to be used to extract data from a struct
  // by calling 'declare_config()'.
  static MetaData create();
  static MetaData& instance();
  ~MetaData();

  // Fields to store the data.
  enum Mode { kUnset, kToString, kCheckValid } mode = Mode::kUnset;
  ValidityChecker validity_checker;
  std::string name = "Unnamed Config";

  // Allow any visitor to access config data.

 private:
  // Create one instance per thread and store the reference to it.
  explicit MetaData(std::thread::id _id);

  // Static registration to get access to the correct instance.
  inline static std::map<std::thread::id, MetaData*> instances;
  inline static std::mutex instance_mutex;

  // Member data.
  const std::thread::id id;
};

}  // namespace config::internal
