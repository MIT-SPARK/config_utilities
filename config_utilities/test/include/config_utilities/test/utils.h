#pragma once

#include <string>

#include <yaml-cpp/yaml.h>

namespace config::test {

std::string getResourcePath();

YAML::Node loadResource(const std::string& name);

}  // namespace config::test
