#include "config_utilities/internal/yaml_parser.h"

namespace config::internal {

std::vector<std::string> YamlParser::children() const {
  if (!node_.IsMap()) {
    return {};
  }

  std::vector<std::string> children;
  for (const auto& kv_pair : node_) {
    children.push_back(kv_pair.first.as<std::string>());
  }

  return children;
}

std::string YamlParser::fromYamlImpl(uint8_t& value, const YAML::Node& node) const {
  value = node_.as<uint16_t>();
  return std::string();
}

std::string YamlParser::toYamlImpl(const std::string& name, const uint8_t& value) {
  node_[name] = static_cast<uint16_t>(value);
  return std::string();
}

}  // namespace config::internal
