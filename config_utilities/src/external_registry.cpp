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

std::unique_ptr<ExternalRegistry> ExternalRegistry::s_instance_ = nullptr;

struct ExternalRegistry::RegistryEntry {
  ModuleInfo key;
  std::string type;
  std::string derived;
};

bool operator<(const ExternalRegistry::RegistryEntry& lhs, const ExternalRegistry::RegistryEntry& rhs) {
  return lhs.type == rhs.type ? lhs.key < rhs.key : lhs.type < rhs.type;
}

struct LibraryHolderImpl : LibraryHolder {
  explicit LibraryHolderImpl(const std::filesystem::path& library_path) {
    // These modes are chosen to somewhat closely mimic dlopen. It will both append appropriate prefixes and suffixes
    // (lib and .so for linux) and allows loading from LD_LIBRARY_PATH by just specfiying the library name.
    // Your mileage may vary on windows, though it looks like boost::dll does a good job picking defaults that
    // are equivalent between the platforms.
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
  // TODO(nathan) toggle this via settings
  std::cerr << "[WARNING] Unloading external library: " << library_path << std::endl;
  auto iter = entries_.find(library_path);
  if (iter != entries_.end()) {
    // Remove any factories that use code from the external library in question. This will also delete any factory
    // module maps that become empty (as they rely on types only defined in the plugin). While plugin libraries
    // shouldn't do this, it's easy to happen in practice and will cause segfaults
    for (const auto& entry : iter->second) {
      internal::ModuleRegistry::removeModule(entry.key, entry.type);
      library_lookup_.erase(entry);
    }
  }

  auto instance_iter = instances_.find(library_path);
  if (instance_iter != instances_.end()) {
    for (const auto& internal_ref : instance_iter->second) {
      const auto actual_ref = internal_ref.lock();
      if (actual_ref) {
        actual_ref->cleanup();
      }
    }
  }

  entries_.erase(library_path);
  libraries_.erase(library_path);
  instances_.erase(library_path);
}

LibraryGuard ExternalRegistry::load(const std::filesystem::path& library_path) {
  Logger::logInfo("Loading external library: " + library_path.string());

  // this sets the global registry to do two things:
  //   - it disallows registration of previously registered factories (silently) for registrations linked in the plugin
  //   - it provides a callback with information about everything that can be successfully registered (so that we can
  //   unload it when the library unloads)
  ModuleRegistry::lock([library_path](const auto& key, const auto& type, const auto& derived) {
    ExternalRegistry::registerType(library_path, {key, type, derived});
  });

  // load the library. This has the effect of calling all static initializers in the library (during the loading
  // process), which registers all external factories
  instance().libraries_.emplace(library_path, std::make_unique<LibraryHolderImpl>(library_path));

  // return the global registry to the normal state (erroring on duplicate factories)
  ModuleRegistry::unlock();

  // return a scoped guard that controls when the library is unloaded
  return LibraryGuard(library_path);
}

void ExternalRegistry::registerType(const std::string& current_library, const RegistryEntry& entry) {
  std::stringstream ss;
  ss << "Registering type '" << entry.type << "' with signature " << entry.key.signature() << " for library '"
     << current_library << "'";
  Logger::logWarning(ss.str());

  auto& entries = instance().entries_;
  auto iter = entries.find(current_library);
  if (iter == entries.end()) {
    iter = entries.emplace(current_library, std::vector<RegistryEntry>()).first;
  }

  iter->second.push_back(entry);
  instance().library_lookup_[entry] = current_library;
}

void ExternalRegistry::registerInstance(const RegistryEntry& entry, void* pointer) {
  const auto& library_lookup = instance().library_lookup_;
  auto iter = library_lookup.find(entry);
  if (iter == library_lookup.end()) {
    return;
  }

  std::stringstream ss;
  ss << "Allocated instance of '" << entry.type << "' for " << entry.key.signature() << " @ " << pointer
     << " with library " << iter->second;
  Logger::logWarning(ss.str());
  instance().external_allocations_.emplace(pointer, entry);
}

ExternalRegistry& ExternalRegistry::instance() {
  if (!s_instance_) {
    s_instance_.reset(new ExternalRegistry());
    ModuleRegistry::setCreationCallback([](const ModuleInfo& info, const std::string& type, void* pointer) {
      registerInstance({info, type}, pointer);
    });
  }
  return *s_instance_;
}

std::optional<std::string> ExternalRegistry::getLibraryForAllocation(void* pointer) {
  auto& external_allocations = instance().external_allocations_;
  auto iter = external_allocations.find(pointer);
  if (iter == external_allocations.end()) {
    return std::nullopt;
  }

  // TODO(nathan) validate types
  /*
  const auto base_type = internal::typeName<T>();
  if (base_type != iter->second.key.base_type) {
    Logger::logError("Invalid conversion between '" + base_type + "' and registered " + iter->second.signature());
  }
  */

  const auto& library_lookup = instance().library_lookup_;
  auto library_iter = library_lookup.find(iter->second);
  if (library_iter == library_lookup.end()) {
    return std::nullopt;
  }

  external_allocations.erase(iter);
  return library_iter->second;
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
