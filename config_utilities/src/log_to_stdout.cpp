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

#include "config_utilities/logging/log_to_stdout.h"

#include <exception>
#include <iostream>

namespace config::internal {

StdoutLogger::StdoutLogger(Severity min_severity, Severity stderr_severity)
    : min_severity_(min_severity), stderr_severity_(stderr_severity) {}

void StdoutLogger::logImpl(const Severity severity, const std::string& message) {
  if (severity < min_severity_ && severity != Severity::kFatal) {
    return;
  }

  std::stringstream ss;
  switch (severity) {
    case Severity::kInfo:
      ss << "[INFO] " << message;
      break;

    case Severity::kWarning:
      ss << "\033[33m[WARNING] " << message << "\033[0m";
      break;

    case Severity::kError:
      ss << "\033[31m[ERROR] " << message << "\033[0m";
      break;

    case Severity::kFatal:
      throw std::runtime_error(message);
  }

  if (severity < stderr_severity_) {
    std::cout << ss.str() << std::endl;
  } else {
    std::cerr << ss.str() << std::endl;
  }
}

StdoutLogger::Initializer::Initializer() { Logger::setLogger(std::make_shared<StdoutLogger>()); }

}  // namespace config::internal
