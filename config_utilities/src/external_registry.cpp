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
namespace internal {

struct ExternalRegistry::RegistryEntry {
  ModuleInfo key;
  std::string type;
  std::string derived;
};

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

struct LibraryHolderImpl : LibraryHolder {
  explicit LibraryHolderImpl(const std::filesystem::path& library_path) {
    const auto mode = boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    library = boost::dll::shared_library(library_path.string(), mode);
  }

  boost::dll::shared_library library;
};

LibraryGuard::LibraryGuard() {}

LibraryGuard::LibraryGuard(const std::filesystem::path library) : library_(library) {}

LibraryGuard::~LibraryGuard() { unload(); }

LibraryGuard::LibraryGuard(LibraryGuard&& other) : library_(std::exchange(other.library_, "")) {}

LibraryGuard& LibraryGuard::operator=(LibraryGuard&& other) {
  library_ = std::exchange(other.library_, "");
  return *this;
}

LibraryGuard::operator bool() const { return !library_.empty(); }

void LibraryGuard::unload() {
  if (!library_.empty()) {
    ExternalRegistry::instance().unload(library_);
  }
  library_.clear();
}

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
  Logger::logInfo("Unloading " + library_path.string());
  auto iter = entries_.find(library_path);
  if (iter != entries_.end()) {
    for (const auto& entry : iter->second) {
      internal::ModuleRegistry::removeModule(entry.key, entry.type);
    }
  }

  entries_.erase(library_path);
  libraries_.erase(library_path);
}

LibraryGuard ExternalRegistry::load(const std::filesystem::path& library_path) {
  ModuleRegistry::lock([library_path](const auto& key, const auto& type, const auto& derived) {
    ExternalRegistry::registerType(library_path, {key, type, derived});
  });

  instance().libraries_.emplace(library_path, std::make_unique<LibraryHolderImpl>(library_path));
  ModuleRegistry::unlock();

  return LibraryGuard(library_path);
}

void ExternalRegistry::registerType(const std::string& current_library, const RegistryEntry& entry) {
  auto& entries = instance().entries_;
  auto iter = entries.find(current_library);
  if (iter == entries.end()) {
    iter = entries.emplace(current_library, std::vector<RegistryEntry>()).first;
  }

  iter->second.push_back(entry);
}

ExternalRegistry& ExternalRegistry::instance() {
  static ExternalRegistry s_instance;
  return s_instance;
}

}  // namespace internal

internal::LibraryGuard loadExternalFactories(const std::filesystem::path& library_path) {
  return internal::ExternalRegistry::load(library_path);
}

internal::LibraryGuard::List loadExternalFactories(const std::vector<std::filesystem::path>& libraries) {
  internal::LibraryGuard::List guards;
  for (const auto& library_path : libraries) {
    guards.push_back(internal::ExternalRegistry::load(library_path));
  }

  return guards;
}

}  // namespace config
