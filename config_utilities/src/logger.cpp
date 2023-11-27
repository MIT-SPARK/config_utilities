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

#include "config_utilities/internal/logger.h"

#include <stdexcept>

namespace config::internal {

std::string severityToString(const Severity severity) {
  switch (severity) {
    case Severity::kInfo:
      return "Info";
    case Severity::kWarning:
      return "Warning";
    case Severity::kError:
      return "Error";
    case Severity::kFatal:
      return "Fatal";
  }
  return "UNKNOWN";
}

void Logger::log(const Severity severity, const std::string& message) { logger_->logImpl(severity, message); }

void Logger::logInfo(const std::string& message) { logger_->logImpl(Severity::kInfo, message); }

void Logger::logWarning(const std::string& message) { logger_->logImpl(Severity::kWarning, message); }

void Logger::logError(const std::string& message) { logger_->logImpl(Severity::kError, message); }

void Logger::logFatal(const std::string& message) { logger_->logImpl(Severity::kFatal, message); }

void Logger::setLogger(Logger::Ptr logger) {
  if (logger) {
    logger_ = std::move(logger);
  }
}

void Logger::logImpl(const Severity severity, const std::string& message) {
  // Empty logger does not log anything. Implementations of logFatal are expected to stop execution of the program.
  if (severity == Severity::kFatal) {
    throw std::runtime_error("FATAL: " + message);
  }
}

}  // namespace config::internal
