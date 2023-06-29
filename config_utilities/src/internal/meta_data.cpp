#include "config_utilities/internal/meta_data.h"

#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include <glog/logging.h>

namespace config::internal {

MetaData MetaData::create() {
  const std::thread::id id = std::this_thread::get_id();
  return MetaData(id);
}

MetaData& MetaData::instance() {
  std::lock_guard<std::mutex> lock(instance_mutex);
  const std::thread::id id = std::this_thread::get_id();
  auto it = instances.find(id);
  if (it == instances.end()) {
    // This should never happen as operators are managed internally. Caught here for debugging.
    LOG(FATAL) << "MetaData for thread " << id << " accessed but was not created.";
  }
  return *instances.at(id);
}

MetaData::~MetaData() {
  std::lock_guard<std::mutex> lock(instance_mutex);
  instances.erase(id);
};

// Create one instance per thread and store the reference to it.
MetaData::MetaData(std::thread::id _id) : id(_id) {
  std::lock_guard<std::mutex> lock(instance_mutex);
  if (instances.find(id) != instances.end()) {
    // This should never happen as operators are managed internally. Caught here for debugging.
    LOG(FATAL) << "Tried to create MetaData for thread " << id << " which already exists.";
  }
  instances[id] = this;
}

}  // namespace config::internal
