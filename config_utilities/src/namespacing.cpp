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

#include "config_utilities/internal/namespacing.h"

#include "config_utilities/internal/logger.h"
#include "config_utilities/internal/string_utils.h"
#include "config_utilities/internal/visitor.h"

namespace config {

NameSpace::NameSpace(const std::string& name_space) : sub_ns_(name_space) { enter(); }

NameSpace::~NameSpace() {
  if (is_open_ && internal::Visitor::hasInstance()) {
    exit();
  }
}

void NameSpace::enter() {
  if (is_open_) {
    internal::Logger::logWarning("NameSpace::enter() called on already open namespace.");
    return;
  }

  internal::Visitor& visitor = internal::Visitor::instance();
  previous_ns_ = visitor.name_space;
  visitor.name_space = internal::joinNamespace(previous_ns_, sub_ns_);
  is_open_ = true;
}

void NameSpace::exit() {
  if (!is_open_) {
    internal::Logger::logWarning("NameSpace::exit() called on already closed namespace.");
    return;
  }

  // Verify that this namespace has not been implicitly closed by a previous sub-namespace.
  internal::Visitor& visitor = internal::Visitor::instance();
  if (visitor.name_space.find(previous_ns_) != 0) {
    // NOTE(lschmid): This results still in valid namespace behavior, but we warn the user that this is not intended.
    internal::Logger::logWarning("NameSpace::exit() called on namespace that was implicitly closed.");
    return;
  }

  // NOTE(lschmid): If the namespace is exited in a different order than it was entered, this simply closes all
  // namespaces that were opened later which is ok behavior.
  visitor.name_space = previous_ns_;
  is_open_ = false;
}

void enter_namespace(const std::string& name_space) {
  internal::Visitor::instance().open_namespaces.emplace_back(std::make_unique<internal::OpenNameSpace>(name_space));
}

void exit_namespace() {
  internal::Visitor& visitor = internal::Visitor::instance();

  if (visitor.open_namespaces.empty()) {
    internal::Logger::logWarning("exit_namespace() called on empty namespace stack.");
    return;
  }
  if (!visitor.open_namespaces.back()->isLocked()) {
    visitor.open_namespaces.pop_back();
  }
}

void clear_namespaces() {
  internal::Visitor& visitor = internal::Visitor::instance();
  while (!visitor.open_namespaces.empty() && !visitor.open_namespaces.back()->isLocked()) {
    visitor.open_namespaces.pop_back();
  }
}

void switch_namespace(const std::string& name_space) {
  exit_namespace();
  enter_namespace(name_space);
}

std::string current_namespace() { return internal::Visitor::instance().name_space; }

internal::OpenNameSpace::OpenNameSpace(const std::string& name_space) : ns(name_space) {}

void internal::OpenNameSpace::lock() { ++locks; }

void internal::OpenNameSpace::unlock() {
  if (locks > 0) {
    --locks;
  }
}

bool internal::OpenNameSpace::isLocked() const { return locks > 0; }

void internal::OpenNameSpace::performOperationWithGuardedNs(Stack& stack, const std::function<void()>& operation) {
  // Lock each namespace in the stack. This should prevent namespace operations from changing existing namespaces.
  for (const auto& ns : stack) {
    ns->lock();
  }

  // Perform the operation.
  operation();

  // Unlock each namespace in the stack.
  for (const auto& ns : stack) {
    ns->unlock();
  }

  // Remove all trailing namespaces that are not locked anymore.
  while (!stack.empty() && !stack.back()->isLocked()) {
    stack.pop_back();
  }
}

}  // namespace config
