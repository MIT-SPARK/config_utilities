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

#include "config_utilities/settings.h"

#include <memory>

#include "config_utilities/config.h"
#include "config_utilities/factory.h"
#include "config_utilities/internal/formatter.h"
#include "config_utilities/internal/logger.h"

namespace config::internal {

Settings Settings::instance_;

Settings& Settings::instance() { return instance_; }

bool Settings::Introspection::enabled() const { return !output.empty(); }

void Settings::setLogger(const std::string& name) {
  if (name == "none") {
    Logger::setLogger(std::make_unique<Logger>());
    return;
  }
  std::unique_ptr<Logger> new_logger = create<Logger>(name);
  if (new_logger) {
    Logger::setLogger(std::move(new_logger));
  }
}

void Settings::setFormatter(const std::string& name) {
  if (name == "none") {
    Formatter::setFormatter(std::make_unique<Formatter>());
    return;
  }
  std::unique_ptr<Formatter> new_formatter = create<Formatter>(name);
  if (new_formatter) {
    Formatter::setFormatter(std::move(new_formatter));
  }
}

void declare_config(Settings& config) {
  name("Settings");
  field(config.printing, "printing");
  field(config.factory, "factory");
  field(config.external_libraries, "external_libraries");
  field(config.introspection, "introspection");
  field(config.disable_default_stdout_logger, "disable_default_stdout_logger");
}

void declare_config(Settings::Printing& config) {
  name("Printing");
  field(config.width, "width");
  field(config.indent, "indent");
  field(config.subconfig_indent, "subconfig_indent");
  field(config.show_defaults, "show_defaults");
  field(config.show_units, "show_units");
  field(config.inline_subconfigs, "inline_subconfigs");
  field(config.reformat_floats, "reformat_floats");
  field(config.show_missing, "show_missing");
  field(config.show_subconfig_types, "show_subconfig_types");
  field(config.show_virtual_configs, "show_virtual_configs");
  field(config.show_num_checks, "show_num_checks");
  field(config.print_meta_fields, "print_meta_fields");
}

void declare_config(Settings::Factory& config) {
  name("Factory");
  field(config.type_param_name, "type_param_name");
}

void declare_config(Settings::ExternalLibraries& config) {
  name("ExternalLibraries");
  field(config.enabled, "enabled");
  field(config.verbose_load, "verbose_load");
  field(config.log_allocation, "log_allocation");
}

void declare_config(Settings::Introspection& config) {
  name("Introspection");
  field(config.output, "output");
}

}  // namespace config::internal
