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

#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/visitor.h"
#include "config_utilities/settings.h"
#include "config_utilities/traits.h"

namespace config {

namespace internal {

bool hasNoInvalidChecks(const MetaData& data);

}  // namespace internal

/**
 * @brief Check if a config is valid.
 *
 * @tparam ConfigT The config type.
 * @param config The config to check.
 * @param print_warnings Whether to print warnings if the config is not valid. This is off by default.
 * @returns True if the config is valid, false otherwise.
 */
template <typename ConfigT>
bool isValid(const ConfigT& config, bool print_warnings = false) {
  static_assert(isConfig<ConfigT>(),
                "Can not use 'config::isValid()' on non-config type. Please implement 'void declare_config(ConfigT&)' "
                "for your struct.");
  internal::MetaData data = internal::Visitor::getChecks(config);

  if (internal::hasNoInvalidChecks(data)) {
    return true;
  }
  if (print_warnings) {
    internal::Logger::logWarning(
        internal::Formatter::formatErrors(data, "Invalid config", internal::Severity::kWarning));
  }
  return false;
}

/**
 * @brief Assert that a config is valid. This will terminate the program if invalid.
 *
 * @tparam ConfigT The config type.
 * @param config The config to check.
 */
template <typename ConfigT>
const ConfigT& checkValid(const ConfigT& config) {
  static_assert(
      isConfig<ConfigT>(),
      "Can not use 'config::checkValid()' on non-config type. Please implement 'void declare_config(ConfigT&)' "
      "for your struct.");
  internal::MetaData data = internal::Visitor::getChecks(config);

  if (internal::hasNoInvalidChecks(data)) {
    return config;
  }

  internal::Logger::logFatal(internal::Formatter::formatErrors(data, "Invalid config", internal::Severity::kFatal));
  return config;
}

}  // namespace config
