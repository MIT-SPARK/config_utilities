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
#include <utility>
#include <vector>

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/meta_data.h"

namespace config::internal {

/**
 * @brief Abstract interface class for formatters. Formatters implement these methods to format the configs for
 * toString() calls and printing of validity checks.
 */
class Formatter {
 public:
  using Ptr = std::shared_ptr<Formatter>;

  // Constructor and destructor.
  Formatter() = default;
  virtual ~Formatter() = default;

  // Accessing the formatter.
  // Format all errors in the meta data into the display string.
  static std::string formatErrors(const MetaData& data,
                                  const std::string& what = "",
                                  const Severity severity = Severity::kWarning);

  // Format all missing fields in the meta data into the display string.
  static std::string formatMissing(const MetaData& data,
                                   const std::string& what = "",
                                   const Severity severity = Severity::kWarning);

  // Format the content of a single config the display string.
  static std::string formatConfig(const MetaData& data);

  // Format the content of multiple configs the display string.
  static std::string formatConfigs(const std::vector<MetaData>& data);

  // Set the global formatter.
  static void setFormatter(Formatter::Ptr formatter);

 protected:
  virtual std::string formatErrorsImpl(const MetaData& data, const std::string& what, const Severity severity);
  virtual std::string formatMissingImpl(const MetaData& data, const std::string& what, const Severity severity);
  virtual std::string formatConfigImpl(const MetaData& data);
  virtual std::string formatConfigsImpl(const std::vector<MetaData>& data);

 private:
  std::string getUnspecifiedString() const;
  inline static Formatter::Ptr formatter_ = std::make_shared<Formatter>();
};

}  // namespace config::internal
