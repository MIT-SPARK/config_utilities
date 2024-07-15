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

#include "config_utilities/internal/formatter.h"

namespace config::internal {

std::string Formatter::formatErrors(const MetaData& data, const std::string& what, const Severity severity) {
  return formatter_->formatErrorsImpl(data, what, severity);
}

std::string Formatter::formatMissing(const MetaData& data, const std::string& what, const Severity severity) {
  return formatter_->formatMissingImpl(data, what, severity);
}

std::string Formatter::formatConfig(const MetaData& data) { return formatter_->formatConfigImpl(data); }

std::string Formatter::formatConfigs(const std::vector<MetaData>& data) { return formatter_->formatConfigsImpl(data); }

void Formatter::setFormatter(Formatter::Ptr formatter) {
  if (formatter) {
    formatter_ = std::move(formatter);
  }
}

std::string Formatter::formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity) {
  return getUnspecifiedString();
}

std::string Formatter::formatMissingImpl(const MetaData& data, const std::string& what, const Severity severity) {
  return getUnspecifiedString();
}

std::string Formatter::formatConfigImpl(const MetaData& data) { return getUnspecifiedString(); }

std::string Formatter::formatConfigsImpl(const std::vector<MetaData>& data) { return getUnspecifiedString(); }

std::string Formatter::getUnspecifiedString() const {
  return "No format specified. Specify a format by including one of 'config_utilities/formatters/<preferred_style>.h'.";
}

}  // namespace config::internal
