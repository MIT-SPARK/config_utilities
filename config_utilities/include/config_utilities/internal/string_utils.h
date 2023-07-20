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
 * @returns The centered string.
 */
std::string printCenter(const std::string& text, int width, char symbol);

/**
 * @brief Split a namespace string into a vector of all non-empty strings separated by a delimiter.
 *
 * @param text The text to be splitNamespace.
 * @param delimiter The delimiter to splitNamespace the text at.
 * @returns The vector of strings.
 */
std::vector<std::string> splitNamespace(const std::string& text, const std::string& delimiter = "/");

/**
 * @brief Join a vector of strings into a single string separated by a delimiter.
 */
std::string joinNamespace(const std::vector<std::string>& namespaces, const std::string& delimiter = "/");

/**
 * @brief Join two namespace strings into a single namespace string separated by a delimiter.
 */
std::string joinNamespace(const std::string& namespace_1,
                          const std::string& namespace_2,
                          const std::string& delimiter = "/");

/**
 * @brief Formatting of YAML nodes to strings. Most config types can be neatly represented as low-depth yaml nodes, or
 * should otherwise probably be wrapped in a separate confi struct.
 *
 * @param data The data to be formatted.
 * @returns The formatted string.
 */
std::string dataToString(const YAML::Node& data);

/**
 * @brief Find all occurences of a substring in a string.
 *
 * @param text The text to be searched.
 * @param substring The substring to be searched for.
 * @returns The vector of positions of the substring in the string.
 */
std::vector<size_t> findAllSubstrings(const std::string& text, const std::string& substring);

}  // namespace config::internal
