

#include "config_utilities/internal/visitor.h"

#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/settings.h"

namespace config::internal {

Visitor::~Visitor() {
  std::lock_guard<std::mutex> lock(instance_mutex);
  instances.erase(id);
}

Visitor Visitor::create() {
  const std::thread::id id = std::this_thread::get_id();
  return Visitor(id);
}

Visitor& Visitor::instance() {
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

void Visitor::visitName(const std::string& name) { Visitor::instance().data.name = name; }

void Visitor::visitCheckCondition(bool condition, const std::string& error_message) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.checker.checkCondition(condition, error_message);
}

std::optional<YAML::Node> Visitor::visitVariableConfig(bool is_set, bool is_optional, const std::string& type) {
  Visitor& visitor = Visitor::instance();
  visitor.data.is_variable_config = true;

  if (visitor.mode == Visitor::Mode::kCheck) {
    if (!is_set && !is_optional) {
      // The config is required and not set.
      visitor.checker.checkCondition(
          false, "Variable config '" + visitor.data.current_field_name + "' is required but not set.");
    }
  }

  if (visitor.mode == Visitor::Mode::kGet) {
    if (is_set) {
      // Also write the type param back to file.
      visitor.parser.toYaml(
          Settings::instance().factory_type_param_name, type, visitor.name_space, visitor.name_prefix);
    }
  }

  if (visitor.mode == Visitor::Mode::kSet) {
    // Return the data to intialize the variable config if this is the first time setting it.
    return lookupNamespace(visitor.parser.node(), visitor.name_space);
  }

  return std::nullopt;
}

}  // namespace config::internal
