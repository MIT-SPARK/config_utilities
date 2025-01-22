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

#include <vector>

#include <config_utilities/internal/logger.h>
#include <config_utilities/internal/visitor.h>
#include <config_utilities/internal/yaml_parser.h>

namespace config {

/**
 * @brief Lists all the fields of the given configuration.
 *
 * This function retrieves metadata from the provided configuration object
 * and extracts the names of all fields, returning them in a vector.
 *
 * @note This function does not list fields of nested and sub-configurations.
 * @tparam ConfigT The type of the configuration.
 * @param config The configuration object whose fields are to be listed.
 * @return A vector containing the names of all fields in the configuration.
 */
template <typename ConfigT>
std::vector<std::string> listFields(const ConfigT& config) {
  internal::MetaData data = internal::Visitor::getValues(config);
  std::vector<std::string> fields;
  for (const auto& field_info : data.field_infos) {
    fields.emplace_back(field_info.name);
  }
  return fields;
}

/**
 * @brief Retrieves the value of a specified field from the given configuration.
 *
 * This function searches for a field with the specified name in the provided
 * configuration object and attempts to convert its value to the requested type.
 * If the field is found and the conversion is successful, the value is returned
 * as an optional. If the field is not found or the conversion fails, a warning
 * is logged and an empty optional is returned.
 *
 * @tparam ConfigT The type of the configuration.
 * @tparam T The type to which the field value should be converted.
 * @param config The configuration object from which the field value is to be retrieved.
 * @param field_name The name of the field whose value is to be retrieved.
 * @return An optional containing the value of the field if found and successfully converted,
 *         otherwise an empty optional.
 */
template <typename ConfigT, typename T>
std::optional<T> getField(const ConfigT& config, const std::string& field_name) {
  internal::MetaData data = internal::Visitor::getValues(config);
  for (const auto& field_info : data.field_infos) {
    if (field_info.name == field_name) {
      std::string error;
      std::optional<T> value = internal::YamlParser::fromYaml<T>(field_info.value, &error);
      if (!error.empty()) {
        internal::Logger::logWarning("Field '" + field_name +
                                     "' could not be converted to the requested type: " + error);
      }
      return value;
    }
  }
  internal::Logger::logWarning("Field '" + field_name + "' not found in config.");
  return std::nullopt;
}

}  // namespace config