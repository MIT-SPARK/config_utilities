

#include "config_utilities/internal/visitor.h"

namespace config::internal {

Visitor::~Visitor() {
  std::lock_guard<std::mutex> lock(instance_mutex);
  instances.erase(id);
}

Visitor Visitor::create() {
  const std::thread::id id = std::this_thread::get_id();
  return Visitor(id);
}

Visitor::Visitor(std::thread::id _id) : id(_id) {
  // Create one instance per thread and store the reference to it.
  std::lock_guard<std::mutex> lock(instance_mutex);
  if (instances.find(id) != instances.end()) {
    // This should never happen as  meta data are managed internally. Caught here for debugging.
    std::stringstream ss;
    ss << "Tried to create Visitor for thread " << id << " which already exists.";
    throw std::runtime_error(ss.str());
  }
  instances[id] = this;
}

void Visitor::extractErrors() {
  // Move the errors from the parser and checker to the meta data.
  data.errors.insert(data.errors.end(), parser.getErrors().begin(), parser.getErrors().end());
  data.errors.insert(data.errors.end(), checker.getWarnings().begin(), checker.getWarnings().end());
  checker.resetWarnings();
  parser.resetErrors();
}

}  // namespace config::internal
