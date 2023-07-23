#include "config_utilities/internal/yaml_parser.h"

namespace config::internal {

void YamlParser::fromYamlImpl(uint8_t& value, const YAML::Node& node, std::string& error) {
  const int int_val = node.as<int>();
  const int max = std::numeric_limits<uint8_t>::max();
  const int min = std::numeric_limits<uint8_t>::lowest();
  if (int_val > max) {
    std::stringstream ss;
    ss << "Value '" << int_val << "' overflows storage max of '" << max << "'.";
    error = ss.str();
    return;
  }
  if (int_val < min) {
    std::stringstream ss;
    ss << "Value '" << int_val << "' underflows storage min of '" << min << "'.";
    error = ss.str();
    return;
  }
  value = node.as<uint16_t>();
}

YAML::Node YamlParser::toYamlImpl(const std::string& name, const uint8_t& value, std::string& error) {
  YAML::Node node;
  node[name] = static_cast<uint16_t>(value);
  return node;
}

}  // namespace config::internal
