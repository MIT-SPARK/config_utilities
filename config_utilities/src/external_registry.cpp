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

#include <iostream>

#include <boost/dll.hpp>
#include <config_utilities/factory.h>

namespace config {
namespace internal {

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

struct LibraryHolder {
  virtual ~LibraryHolder() = default;
};

struct LibraryHolderImpl : LibraryHolder {
  explicit LibraryHolderImpl(const std::filesystem::path& library_path) {
    const auto mode = boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    library = boost::dll::shared_library(library_path.string(), mode);
  }

  boost::dll::shared_library library;
};

struct ExternalRegistry {
  struct RegistryEntry {
    ModuleInfo key;
    std::string type;
    std::string derived;
  };

  ~ExternalRegistry();

  void unload(const std::filesystem::path& library_path);
  static void load(const std::filesystem::path& library_path);
  static void registerType(const std::string& current_library,
                           const ModuleInfo& info,
                           const std::string& type,
                           const std::string& derived);

  static ExternalRegistry& instance();

 private:
  ExternalRegistry() = default;

  std::map<std::string, std::unique_ptr<LibraryHolder>> libraries_;
  std::map<std::string, std::vector<RegistryEntry>> entries_;
};

ExternalRegistry::~ExternalRegistry() {
  std::vector<std::string> libraries;
  for (const auto& [path, lib] : libraries_) {
    libraries.push_back(path);
  }

  for (const auto& path : libraries) {
    unload(path);
  }
}

void ExternalRegistry::unload(const std::filesystem::path& library_path) {
  std::cout << "Unloading " << library_path << std::endl;
  auto iter = entries_.find(library_path);
  if (iter != entries_.end()) {
    for (const auto& entry : iter->second) {
      std::cout << "Unloading " << entry.type << " for " << entry.key.signature() << std::endl;
      internal::ModuleRegistry::removeModule(entry.key, entry.type);
    }
  }

  entries_.erase(library_path);
  libraries_.erase(library_path);
}

void ExternalRegistry::load(const std::filesystem::path& library_path) {
  ModuleRegistry::lock([library_path](const auto& info, const auto& type, const auto& derived) {
    ExternalRegistry::registerType(library_path, info, type, derived);
  });

  instance().libraries_.emplace(library_path, std::make_unique<LibraryHolderImpl>(library_path));
  ModuleRegistry::unlock();
}

void ExternalRegistry::registerType(const std::string& current_library,
                                    const ModuleInfo& info,
                                    const std::string& type,
                                    const std::string& derived) {
  internal::Logger::logInfo("type: '" + type + "', info: " + info.signature() + ", derived: '" + derived + "'");

  auto& entries = instance().entries_;
  auto iter = entries.find(current_library);
  if (iter == entries.end()) {
    iter = entries.emplace(current_library, std::vector<RegistryEntry>()).first;
  }

  iter->second.push_back({info, type, derived});
}

ExternalRegistry& ExternalRegistry::instance() {
  static ExternalRegistry s_instance;
  return s_instance;
}

}  // namespace internal

void loadExternalFactories(const std::filesystem::path& library_path, const std::string& registry_name) {
  // execution:
  // - ModuleRegistry is informed that an external library is about to be loaded
  // - We load the shared library and the static library initializers trigger
  //   - Every registration in the external library will trigger
  //   - Registrations will go through one of three factories
  //     - Config factory registers type which has different instances in different shared objects

  internal::ExternalRegistry::load(library_path);
  internal::Logger::logInfo(internal::ModuleRegistry::getAllRegistered());
  auto test_logger = internal::ObjectFactory<internal::Logger>::create("test_logger");
  test_logger.reset();
  internal::ExternalRegistry::instance().unload(library_path);
  internal::Logger::logInfo(internal::ModuleRegistry::getAllRegistered());
}

}  // namespace config
