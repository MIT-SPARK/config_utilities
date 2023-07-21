
#include "config_utilities/internal/namespacing.h"

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"

namespace config {

NameSpace::NameSpace(const std::string& name_space) : sub_ns_(name_space) { enter(); }

NameSpace::~NameSpace() {
  if (is_open_ && internal::Visitor::hasInstance()) {
    exit();
  }
}

void NameSpace::enter() {
  if (is_open_) {
    internal::Logger::logWarning("NameSpace::enter() called on already open namespace.");
    return;
  }

  internal::Visitor& visitor = internal::Visitor::instance();
  previous_ns_ = visitor.name_space;
  visitor.name_space = internal::joinNamespace(previous_ns_, sub_ns_);
  is_open_ = true;
}

void NameSpace::exit() {
  if (!is_open_) {
    internal::Logger::logWarning("NameSpace::exit() called on already closed namespace.");
    return;
  }

  // Verify that this namespace has not been implicitly closed by a previous sub-namespace.
  internal::Visitor& visitor = internal::Visitor::instance();
  if (visitor.name_space.find(previous_ns_) != 0) {
    // NOTE(lschmid): This results still in valid namespace behavior, but we warn the user that this is not intended.
    internal::Logger::logWarning("NameSpace::exit() called on namespace that was implicitly closed.");
    return;
  }

  // NOTE(lschmid): If the namespace is exited in a different order than it was entered, this simply closes all
  // namespaces that were opened later which is ok behavior.
  visitor.name_space = previous_ns_;
  is_open_ = false;
}

void enter_namespace(const std::string& name_space) {
  internal::Visitor::instance().open_namespaces.emplace_back(name_space);
}

void exit_namespace() {
  internal::Visitor& visitor = internal::Visitor::instance();

  if (visitor.open_namespaces.empty()) {
    internal::Logger::logWarning("exit_namespace() called on empty namespace stack.");
    return;
  }
  visitor.open_namespaces.pop_back();
}

void clear_namespaces() {
  internal::Visitor& visitor = internal::Visitor::instance();
  while (!visitor.open_namespaces.empty()) {
    visitor.open_namespaces.pop_back();
  }
}

void switch_namespace(const std::string& name_space) {
  exit_namespace();
  enter_namespace(name_space);
}

}  // namespace config
