

#include "config_utilities/internal/visitor.h"

#include "config_utilities/internal/yaml_utils.h"
#include "config_utilities/settings.h"

namespace config::internal {

thread_local std::vector<Visitor*> Visitor::instances = {};

Visitor::~Visitor() { instances.pop_back(); }

bool Visitor::hasInstance() { return !instances.empty(); }

Visitor::Visitor(Mode _mode, const std::string& _name_space, const std::string& field_name)
    : mode(_mode), name_space(_name_space) {
  data.field_name = field_name;
  // Create instances in a stack per thread and store the reference to it.
  instances.emplace_back(this);
}

Visitor& Visitor::instance() {
  if (instances.empty()) {
    // This should never happen as visitors are managed internally. Caught here for debugging.
    throw std::runtime_error(
        "Visitor instance was accessed but no visitor was created before. Visitor::instance() should only be called "
        "from within a visitor.");
  }
  return *instances.back();
}

void Visitor::visitName(const std::string& name) {
  std::string& current_name = Visitor::instance().data.name;
  // Avoid overriding names. If the name is to be set, it should be cleared previously.
  if (current_name.empty()) {
    current_name = name;
  }
}

void Visitor::visitCheck(const CheckBase& check) {
  Visitor& visitor = Visitor::instance();
  if (visitor.mode != Visitor::Mode::kCheck) {
    return;
  }
  visitor.data.checks.emplace_back(check.clone());
}

std::optional<YAML::Node> Visitor::visitVirtualConfig(bool is_set, bool is_optional, const std::string& type) {
  Visitor& visitor = Visitor::instance();
  visitor.data.is_virtual_config = true;

  // Treat the validity of virtual configs as checks.
  if (visitor.mode == Visitor::Mode::kCheck) {
    if (!is_set && !is_optional) {
      const std::string field_name = visitor.data.field_name.empty() ? "" : "'" + visitor.data.field_name + "' ";
      visitor.data.checks.emplace_back(
          new Check(false, "Virtual config " + field_name + "is not set and not marked optional"));
    } else {
      visitor.data.checks.emplace_back(new Check(true, ""));
    }
  }

  if (visitor.mode == Visitor::Mode::kGet) {
    if (is_set) {
      // Also write the type param back to file.
      std::string error;
      YAML::Node type_node =
          YamlParser::toYaml(Settings::instance().factory_type_param_name, type, visitor.name_space, error);
      mergeYamlNodes(visitor.data.data, type_node);
    }
  }

  if (visitor.mode == Visitor::Mode::kSet) {
    // Return the data to intialize the virtual config if this is the first time setting it.
    return lookupNamespace(visitor.data.data, visitor.name_space);
  }

  return std::nullopt;
}

}  // namespace config::internal