#pragma once

#include <algorithm>
#include <string>

namespace config::internal {

/**
 * @brief Print a string centered in a line.
 */
inline std::string printCenter(const std::string& text, int width, char symbol) {
  int first = std::max((width - static_cast<int>(text.length()) - 2) / 2, 0);
  std::string result = std::string(first, symbol) + " " + text + " ";
  result += std::string(std::max(width - static_cast<int>(result.length()), 0), symbol);
  return result;
}

}  // namespace config::internal
