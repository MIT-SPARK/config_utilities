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

#include "config_utilities/factory.h"

namespace config::internal {

namespace {

std::string showWithFilter(const std::map<ModuleInfo, std::map<std::string, std::string>>& registered,
                           const std::function<bool(const ModuleInfo&)>& filter = {}) {
  std::stringstream ss;
  for (const auto& [key, types] : registered) {
    if (filter && !filter(key)) {
      continue;
    }

    ss << "\n" << key.signature() << ":";
    for (const auto& [type_name, derived_type] : types) {
      ss << "\n  '" << type_name << "' (" << derived_type << ")";
    }

    ss << "\n";
  }
  return ss.str();
}

bool isPlainObject(const ModuleInfo& info) { return !info.skip_first_arg && info.underlying_base.empty(); }

bool isObjectWithConfig(const ModuleInfo& info) { return info.skip_first_arg && info.underlying_base.empty(); }

bool isObjectConfig(const ModuleInfo& info) { return !info.underlying_base.empty(); }

std::string banner(const std::string& msg, int width, char symbol = '#') {
  if (width < 2) {
    width = 2;
  }

  std::string symbol_str(1, symbol);
  const auto center = symbol_str + printCenter(msg, width - 2, ' ') + symbol_str;
  return std::string(center.size(), symbol) + "\n" + center + "\n" + std::string(center.size(), symbol) + "\n";
}

}  // namespace

std::unique_ptr<ModuleRegistry> ModuleRegistry::s_instance_ = nullptr;

std::string ModuleInfo::argumentString(const std::string& separator,
                                       const std::string& wrapper,
                                       const std::string& placeholder) const {
  if (arguments.empty()) {
    return "";
  }

  if (arguments.size() == 1 && skip_first_arg) {
    return placeholder.empty() ? "" : placeholder;
  }

  auto iter = arguments.begin();
  if (skip_first_arg) {
    ++iter;
  }

  std::string arg_list = skip_first_arg && !placeholder.empty() ? placeholder + separator : "";
  while (iter != arguments.end()) {
    arg_list.append(wrapper + *iter + wrapper);
    ++iter;
    if (iter != arguments.end()) {
      arg_list.append(separator);
    }
  }

  return arg_list;
}

std::string ModuleInfo::typeInfo() const {
  return "BaseT='" + base_type + "'" + (underlying_base.empty() ? "" : " (underlying: '" + underlying_base + "')") +
         " and ConstructorArguments={" + argumentString(", ", "'") + "}";
}

std::string ModuleInfo::signature() const {
  return (underlying_base.empty() ? base_type : "Config[" + underlying_base + "]") + "(" + argumentString() + ")";
}

bool operator<(const ModuleInfo& lhs, const ModuleInfo& rhs) {
  if (lhs.base_type == rhs.base_type) {
    return (lhs.underlying_base == rhs.underlying_base) ? lhs.arguments < rhs.arguments
                                                        : lhs.underlying_base < rhs.underlying_base;
  }

  return lhs.base_type < rhs.base_type;
}

bool operator<(const ConfigPair& lhs, const ConfigPair& rhs) {
  return lhs.base_type == rhs.base_type ? lhs.config_type < rhs.config_type : lhs.base_type < rhs.base_type;
}

std::string ModuleRegistry::getAllRegistered() {
  const auto width = Settings::instance().printing.width;
  const auto& registry = instance().type_registry;
  std::stringstream ss;
  ss << banner("Registered Objects", width) << showWithFilter(registry, &isPlainObject) << "\n";
  ss << banner("Registered Objects with Configs", width) << showWithFilter(registry, &isObjectWithConfig) << "\n";
  ss << banner("Registered Configs", width) << showWithFilter(registry, &isObjectConfig) << "\n";
  return ss.str();
}

bool ModuleRegistry::hasModule(const ModuleInfo& key, const std::string& type) {
  const auto& modules = instance().modules;
  const auto iter = modules.find(key);
  if (iter == modules.end()) {
    return false;
  }

  return iter->second->hasEntry(type);
}

void ModuleRegistry::removeModule(const ModuleInfo& key, const std::string& type) {
  {  // module scope
    auto& modules = instance().modules;
    const auto iter = modules.find(key);
    if (iter != modules.end()) {
      iter->second->removeEntry(type);
      if (iter->second->empty()) {
        modules.erase(iter);
      }
    }
  }

  {  // registry scope
    auto& registry = instance().type_registry;
    const auto iter = registry.find(key);
    if (iter != registry.end()) {
      iter->second.erase(type);
      if (iter->second.empty()) {
        registry.erase(iter);
      }
    }
  }
}

std::string ModuleRegistry::getRegistered(const ModuleInfo& key) {
  const auto& registry = instance().type_registry;
  auto iter = registry.find(key);
  if (iter == registry.end()) {
    return "";
  }

  std::string module_list;
  auto miter = iter->second.begin();
  while (miter != iter->second.end()) {
    module_list.append(miter->first);
    ++miter;
    if (miter != iter->second.end()) {
      module_list.append("', '");
    }
  }
  return module_list;
}

void ModuleRegistry::lock(LockCallback func) {
  instance().locked_ = true;
  instance().lock_callback_ = std::move(func);
}

void ModuleRegistry::unlock() {
  instance().locked_ = false;
  instance().lock_callback_ = {};
}

bool ModuleRegistry::locked() { return instance().locked_; }

void ModuleRegistry::setCreationCallback(CreateCallback callback) { instance().create_callback_ = std::move(callback); }

ModuleRegistry& ModuleRegistry::instance() {
  if (!s_instance_) {
    s_instance_.reset(new ModuleRegistry());
  }

  return *s_instance_;
}

// Helper function to read the type param from a node.
bool getTypeImpl(const YAML::Node& data, std::string& type, const std::string& key) {
  if (!data.IsMap()) {
    return false;
  }

  // Get the type or print an error.
  if (!data[key]) {
    return false;
  }

  try {
    type = data[key].as<std::string>();
  } catch (const YAML::Exception& e) {
    return false;
  }

  return true;
}

bool getType(const YAML::Node& data, std::string& type, bool required, const std::string& param_name) {
  const std::string key = param_name.empty() ? Settings::instance().factory.type_param_name : param_name;
  const auto success = getTypeImpl(data, type, key);
  if (!success && required) {
    Logger::logError("Could not read the param '" + key + "' to deduce the type of the module to create.");
  }

  return success;
}

}  // namespace config::internal
