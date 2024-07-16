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

#include <memory>
#include <string>
#include <vector>

#include "config_utilities/factory.h"
#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/meta_data.h"

namespace config::internal {

/**
 * @brief Implements formatting of text in the style of https://github.com/ethz-asl/config_utilities, emphasizing
 * readability wenn printed to the console.
 */
class AslFormatter : public Formatter {
 public:
  AslFormatter() = default;
  ~AslFormatter() override = default;

 protected:
  std::string formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity) override;
  std::string formatMissingImpl(const MetaData& data, const std::string& what, const Severity severity) override;
  std::string formatConfigImpl(const MetaData& data) override;
  std::string formatConfigsImpl(const std::vector<MetaData>& data) override;

 private:
  // Factory registration to allow setting of formatters via Settings::setFormatter().
  inline static const auto registration_ = Registration<Formatter, AslFormatter>("asl");

  // Initialize the asl formatter to be used if included.
  inline static const struct Initializer {
    Initializer() { Formatter::setFormatter(std::make_unique<AslFormatter>()); }
  } initializer_;

  // Helper functions.
  std::string formatErrorsRecursive(const MetaData& data, const std::string& sev, const size_t length);
  std::string formatMissingRecursive(const MetaData& data, const std::string& sev, const size_t length);
  std::string formatChecksInternal(const MetaData& data, const std::string& sev, const size_t length);
  std::string formatErrorsInternal(const MetaData& data, const std::string& sev, const size_t length);
  std::string toStringInternal(const MetaData& data, size_t indent) const;
  std::string formatField(const FieldInfo& info, size_t indent) const;
  std::string formatSubconfig(const MetaData& data, size_t indent) const;
  std::string resolveConfigName(const MetaData& data) const;

  // Formatting options, currently not exposed in global settings but work if want changed.
  // TODO(lschmid): Global formatting options should probably be a config of the formatter.
  // If true add subconfig types after the fieldname.
  constexpr static bool indicate_subconfig_types_ = true;
  // If true label subconfigs as default if all their values are default.
  constexpr static bool indicate_subconfig_default_ = true;
  // If true indicate that a config is a virtual config in the config name.
  constexpr static bool indicate_virtual_configs_ = true;
  // If true indicate the number of a check and total number of checks in failed checks.
  constexpr static bool indicate_num_checks_ = true;

  // Variables.
  std::string name_prefix_;
  size_t total_num_checks_;
  size_t current_check_;
  bool is_first_divider_;
};

}  // namespace config::internal
