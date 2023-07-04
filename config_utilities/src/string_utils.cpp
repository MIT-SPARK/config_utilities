#include "config_utilities/internal/string_utils.h"

#include <algorithm>
#include <sstream>

namespace config::internal {

std::string printCenter(const std::string& text, int width, char symbol) {
  int first = std::max((width - static_cast<int>(text.length()) - 2) / 2, 0);
  std::string result = std::string(first, symbol) + " " + text + " ";
  result += std::string(std::max(width - static_cast<int>(result.length()), 0), symbol);
  return result;
}

std::vector<std::string> split(const std::string& text, const std::string& delimiter) {
  std::vector<std::string> result;
  std::string s = text;
  size_t pos = 0;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    if (pos != 0) {
      result.push_back(text.substr(0, pos));
    }
    s.erase(0, pos + delimiter.length());
  }
  return result;
}

std::string dataToString(const ConfigData& data, const std::string& type_info) {
  return CustomStringConversion::convert(data, type_info);
}

std::string yamlToString(const ConfigData& data) {
  switch (data.Type()) {
    case YAML::NodeType::Scalar:
      return scalarToString(data);
    case YAML::NodeType::Sequence: {
      std::string result = "[";
      for (size_t i = 0; i < data.size(); ++i) {
        result += scalarToString(data[i]);
        if (i < data.size() - 1) {
          result += ", ";
        }
      }
      result += "]";
      return result;
    }
    case YAML::NodeType::Map: {
      std::string result = "{";
      bool has_data = false;
      for (const auto& kv_pair : data) {
        has_data = true;
        result += scalarToString(kv_pair.first) + ": " + scalarToString(kv_pair.second) + ", ";
      }
      if (has_data) {
        result = result.substr(0, result.length() - 2);
      }
      result += "}";
      return result;
    }
    default:
      return kInvalidField;
  }
}

std::string scalarToString(const ConfigData& data) {
  if (!data.IsScalar()) {
    return kInvalidField;
  }
  // Use stringstream to convert to string for any type.
  std::stringstream ss;
  ss << data;
  return ss.str();
}

void CustomStringConversion::addConversion(ConversionFunction conversion) { conversions_.push_back(conversion); }

std::string CustomStringConversion::convert(const ConfigData& data, const std::string& type_info) {
  if (type_info.empty()) {
    // No data is the most common case so use the default conversion.
    return yamlToString(data);
  }
  for (const auto& conversion : conversions_) {
    auto result = conversion(data, type_info);
    if (result) {
      // If any of the conversions feel responsible for the supplied type_info, return the result.
      return *result;
    }
  }

  // Otherwise fall back to the default conversion, which captures all natural YAML types.
  return yamlToString(data);
}

std::vector<CustomStringConversion::ConversionFunction> CustomStringConversion::conversions_;

}  // namespace config::internal
