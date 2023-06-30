#pragma once

#include <map>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

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

}  // namespace config::internal
