#pragma once

#include <cstdint>
#include <string>
#include <type_traits>

namespace config {

/** @brief Conversion to and from char that avoids issues with integer <-> ascii casts
 *
 * This parses the config field to a string before extracting the desired character. When
 * the parsed string ends up being multiple characters, this grabs the first. Note that this
 * disallows non-ascii characters.
 */
struct CharConversion {
  static std::string toIntermediate(char value, std::string& error);
  static char fromIntermediate(const std::string& value, std::string& error);
};

/** @brief Conversion that remaps a specified number of threads to the total number of avaiable cores
 *
 * If the original value is less than or equal to 0, this returns the total number of cores detected,
 * otherwise it returns the originally specified value.
 */
struct ThreadNumConversion {
  static int toIntermediate(int value, std::string& error);
  static int fromIntermediate(int value, std::string& error);
};

}  // namespace config
