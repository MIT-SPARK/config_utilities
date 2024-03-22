#pragma once
#include <string>
#include <exception>

// TODO(nathan) drop once dependencies don't include this

namespace config {
std::string printAllValidConfigs(bool flag) { throw std::runtime_error("deprecated"); }
}  // namespace config
