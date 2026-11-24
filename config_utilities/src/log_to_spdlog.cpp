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

#include <string>
#include "config_utilities/logging/log_to_spdlog.h"

#include <utility>

#ifdef CONFIG_UTILS_ENABLE_SPDLOG_LOGGING
#include <spdlog/spdlog.h>
#include <cstdlib>
#endif

#include "config_utilities/factory.h"
#include "config_utilities/logging/log_to_stdout.h"

namespace config::internal {
namespace {

#ifdef CONFIG_UTILS_ENABLE_SPDLOG_LOGGING
const auto registration = Registration<Logger, SpdlogLogger>("spdlog");
#endif

}  // namespace

SpdlogLogger::SpdlogLogger(std::shared_ptr<spdlog::logger> logger)
    : logger_(std::move(logger)) {
#ifdef CONFIG_UTILS_ENABLE_SPDLOG_LOGGING
  if (!logger_) {
    logger_ = spdlog::default_logger();
  }
#endif
}

void SpdlogLogger::logImpl(Severity severity, const std::string& message) {
#ifdef CONFIG_UTILS_ENABLE_SPDLOG_LOGGING
  switch (severity) {
    case Severity::kInfo:
      logger_->info("{}", message);
      return;
    case Severity::kWarning:
      logger_->warn("{}", message);
      return;
    case Severity::kError:
      logger_->error("{}", message);
      return;
    case Severity::kFatal:
      logger_->critical("{}", message);
      logger_->flush();
      std::abort();
  }
#else
  // Use the existing public dispatch interface for the fallback backend.
  Logger::setLogger(std::make_shared<StdoutLogger>());
  Logger::log(severity, message);
#endif
}

}  // namespace config::internal
