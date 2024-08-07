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

#include "config_utilities/external_registry.h"

#include <boost/dll.hpp>
#include <config_utilities/factory.h>

namespace config {

template <typename T>
struct ManagedInstance {
  void execute(const std::function<void(const T&)>& func) {
    if (!underlying_) {
      return;
    }

    // lock
    func(*underlying_);
    // unlock
  }

  T* underlying_;
};

void loadExternalFactories(const std::filesystem::path& library_path, const std::string& registry_name) {
  // execution:
  // - ModuleRegistry is informed that an external library is about to be loaded
  // - We load the shared library and the static library initializers trigger
  //   - Every registration in the external library will trigger
  //   - Registrations will go through one of three factories
  //     - Config factory registers type which has different instances in different shared objects

  internal::ModuleRegistry::lock();
  const auto mode = boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
  boost::dll::shared_library lib(library_path.string(), mode);

  const auto default_registry_name = library_path.stem().string() + "_registry";
  const auto func = lib.get<void()>(registry_name.empty() ? default_registry_name : registry_name);
  func();

  auto stdout_logge = internal::ObjectFactory<internal::Logger>::create("stdout");

  internal::ModuleRegistry::unlock();
  internal::Logger::logError(internal::ModuleRegistry::getAllRegistered());

  auto test_logger = internal::ObjectFactory<internal::Logger>::create("test_logger");
}

}  // namespace config
