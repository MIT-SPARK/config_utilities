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
 * @brief Formatting of YAML nodes or custom conversions to strings. Separate conversion functions can be implemented
 * for structs that require special formatting by using CustomStringConversion::addConversion(). If no custom conversion
 * is implemented this will fallback to formatting single yaml nodes.
 *
 * @param data The data to be formatted.
 * @param type_info The type info of the data.
 * @return std::string The formatted string.
 */
std::string dataToString(const ConfigData& data, const std::string& type_info);

/**
 * @brief Formatting of single YAML nodes to strings.
 *
 * @param data The data to be formatted.
 * @return std::string The formatted string.
 */
std::string yamlToString(const ConfigData& data);

/**
 * @brief Fformatting of scalar YAML types to strings.
 *
 * @param data The data to be formatted.
 * @return std::string The formatted string.
 */
std::string scalarToString(const ConfigData& data);

/**
 * @brief Registry for custom type conversions.
 */
struct CustomStringConversion {
  /**
   * @brief Function type for custom type conversions.
   * @param const ConfigData&: The data to be converted.
   * @param const std::string&: The type info of the data.
   * @returns std::optional<std::string>: The converted string or std::nullopt if the supplied type info does not match
   * the expected conversion.
   */
  using ConversionFunction = std::function<std::optional<std::string>(const ConfigData&, const std::string&)>;

  static void addConversion(ConversionFunction conversion);
  static std::string convert(const ConfigData& data, const std::string&);

 private:
  static std::vector<ConversionFunction> conversions_;
};

}  // namespace config::internal
