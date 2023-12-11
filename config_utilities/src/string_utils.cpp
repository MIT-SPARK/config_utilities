/** -----------------------------------------------------------------------------
 * Copyright (c) 2023 Massachusetts Institute of Technology.
 * All Rights Reserved.
 *
 * AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
 * AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
 * YEAR:        2023
 * LICENSE:     BSD 3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------------- */

#include "config_utilities/internal/string_utils.h"

#include <algorithm>
#include <regex>
#include <sstream>

namespace config::internal {

std::string printCenter(const std::string& text, int width, char symbol) {
  int first = std::max((width - static_cast<int>(text.length()) - 2) / 2, 0);
  std::string result = std::string(first, symbol) + " " + text + " ";
  result += std::string(std::max(width - static_cast<int>(result.length()), 0), symbol);
  return result;
}

std::vector<std::string> splitNamespace(const std::string& text, const std::string& delimiter) {
  std::vector<std::string> result;
  if (text.empty()) {
    return result;
  }
  std::string s = text;
  size_t pos = 0;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    if (pos != 0) {
      result.push_back(s.substr(0, pos));
    }
    s.erase(0, pos + delimiter.length());
  }
  if (!s.empty()) {
    result.push_back(s);
  }
  return result;
}

std::string joinNamespace(const std::vector<std::string>& namespaces, const std::string& delimiter) {
  std::string result;
  if (namespaces.empty()) {
    return result;
  }
  result = namespaces[0];
  for (size_t i = 1; i < namespaces.size(); ++i) {
    result += delimiter + namespaces[i];
  }
  return result;
}

std::string joinNamespace(const std::string& namespace_1,
                          const std::string& namespace_2,
                          const std::string& delimiter) {
  // Ensure the final formatting is uniform
  std::vector<std::string> ns_1 = splitNamespace(namespace_1, delimiter);
  const std::vector<std::string> ns_2 = splitNamespace(namespace_2, delimiter);
  ns_1.insert(ns_1.end(), ns_2.begin(), ns_2.end());
  return joinNamespace(ns_1, delimiter);
}

std::string scalarToString(const YAML::Node& data, bool reformat_float) {
  std::stringstream orig;
  orig << data;
  if (!reformat_float) {
    return orig.str();
  }

  const std::regex float_detector("[+-]?[0-9]*[.][0-9]+");
  if (!std::regex_search(orig.str(), float_detector)) {
    return orig.str();  // no reason to reformat if no decimal points
  }

  double value;
  try {
    value = data.as<double>();
  } catch (const std::exception&) {
    return orig.str();  // value is some sort of string that can't be parsed as a float
  }

  // this should have default ostream precision for formatting float
  std::stringstream ss;
  ss << value;
  return ss.str();
}

std::string dataToString(const YAML::Node& data, bool reformat_float) {
  switch (data.Type()) {
    case YAML::NodeType::Scalar: {
      // scalars require special handling for float precision
      return scalarToString(data, reformat_float);
    }
    case YAML::NodeType::Sequence: {
      std::string result = "[";
      for (size_t i = 0; i < data.size(); ++i) {
        result += dataToString(data[i], reformat_float);
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
        result +=
            dataToString(kv_pair.first, reformat_float) + ": " + dataToString(kv_pair.second, reformat_float) + ", ";
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

std::vector<size_t> findAllSubstrings(const std::string& text, const std::string& substring) {
  std::vector<size_t> result;
  size_t pos = text.find(substring, 0);
  while (pos != std::string::npos) {
    result.push_back(pos);
    pos = text.find(substring, pos + 1);
  }
  return result;
}

std::string pruneTrailingWhitespace(const std::string& text) {
  size_t i;
  for (i = 0; i < text.size(); ++i) {
    if (text.at(text.size() - i - 1) != ' ') {
      break;
    }
  }

  if (!i) {
    return text;
  }

  return text.substr(0, text.size() - i);
}

std::string pruneLeadingWhitespace(const std::string& text) {
  const size_t pos = text.find_first_not_of(' ');
  if (pos == std::string::npos) {
    return "";
  }
  return text.substr(pos);
}

std::string pruneWhitespace(const std::string& text) { return pruneLeadingWhitespace(pruneTrailingWhitespace(text)); }

std::string wrapString(const std::string& str, size_t width, size_t indent, bool indent_first_line) {
  std::string result;
  std::string remaining = str;
  const size_t length = width - indent;

  // Format the first line indent.
  if (indent_first_line) {
    result = std::string(indent, ' ');
  } else {
    const size_t first_line_length = std::min(indent, str.length());
    result = str.substr(0, first_line_length);
    remaining = str.substr(first_line_length);
  }

  if (remaining.empty()) {
    return pruneTrailingWhitespace(result);
  }

  // Wrap all other lines within the indent range.
  bool is_first_line = true;
  while (!remaining.empty()) {
    std::string next_line = remaining.substr(0, std::min(length, remaining.length()));
    remaining = remaining.substr(next_line.size());

    // Prune leading and trailing whitespace of all lines except the first.
    if (!is_first_line) {
      while (!next_line.empty() && next_line.at(0) == ' ') {
        next_line = next_line.substr(1);
        if (!remaining.empty()) {
          next_line += remaining.at(0);
          remaining = remaining.substr(1);
        }
      }
    }
    next_line = pruneTrailingWhitespace(next_line);
    if (next_line.empty()) {
      return result;
    }

    // Aggregate line.
    if (!is_first_line) {
      result += "\n" + std::string(indent, ' ');
    }
    is_first_line = false;
    result += next_line;
  }

  return result;
}

}  // namespace config::internal
