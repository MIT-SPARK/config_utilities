#include "config_utilities/internal/yaml_parser.h"

namespace config::internal {

std::string YamlParser::fromYamlImpl(uint8_t& value, const YAML::Node& node) const {
  value = node.as<uint16_t>();
  return std::string();
}

std::string YamlParser::toYamlImpl(const std::string& name, const uint8_t& value) {
  node_[name] = static_cast<uint16_t>(value);
  return std::string();
}

}  // namespace config::internal
