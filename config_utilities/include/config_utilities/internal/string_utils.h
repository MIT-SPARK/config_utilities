#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "config_utilities/internal/meta_data.h"

namespace config::internal {

inline static const std::string kInvalidField = "Invalid Field";

/**
 * @brief Print a string centered in a line.
 *
 * @param text The text to be centered.
 * @param width The width of the line.
 * @param symbol The symbol to fill the line with.
 * @return std::string The centered string.
 */
std::string printCenter(const std::string& text, int width, char symbol);

/**
 * @brief Split a string into a vector of strings.
 *
 * @param text The text to be split.
 * @param delimiter The delimiter to split the text at.
 * @return std::vector<std::string> The vector of strings.
 */
std::vector<std::string> split(const std::string& text, const std::string& delimiter);

/**
 * @brief Formatting of YAML nodes to strings. Most config types can be neatly represented as low-depth yaml nodes, or
 * should otherwise probably be wrapped in a separate confi struct.
 *
 * @param data The data to be formatted.
 * @return std::string The formatted string.
 */
std::string dataToString(const ConfigData& data);

/**
 * @brief Find all occurences of a substring in a string.
 *
 * @param text The text to be searched.
 * @param substring The substring to be searched for.
 * @return std::vector<size_t> The vector of positions of the substring in the string.
 */
std::vector<size_t> findAllSubstrings(const std::string& text, const std::string& substring);

}  // namespace config::internal
