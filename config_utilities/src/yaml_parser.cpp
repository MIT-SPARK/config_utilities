#include "config_utilities/internal/yaml_parser.h"

namespace config::internal {

void YamlParser::fromYamlImpl(uint8_t& value, const YAML::Node& node, std::string& error) const {
  value = node.as<uint16_t>();
}

YAML::Node YamlParser::toYamlImpl(const std::string& name, const uint8_t& value, std::string& error) {
  YAML::Node node;
  node[name] = static_cast<uint16_t>(value);
  return node;
}

}  // namespace config::internal
