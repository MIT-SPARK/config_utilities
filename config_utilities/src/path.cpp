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

#include "config_utilities/types/path.h"

namespace config {

std::string Path::toIntermediate(const std::filesystem::path& value, std::string&) {
  return value.lexically_normal().string();
}

void Path::fromIntermediate(const std::string& intermediate, std::filesystem::path& value, std::string&) {
  value = std::filesystem::path(intermediate).lexically_normal();
}

std::string Path::toIntermediate(std::string value, std::string&) {
  return std::filesystem::path(value).lexically_normal().string();
}

void Path::fromIntermediate(const std::string& intermediate, std::string& value, std::string&) {
  value = std::filesystem::path(intermediate).lexically_normal().string();
}

bool Path::IsSet::valid() const { return !path_.empty(); }

std::string Path::IsSet::message() const { return "path is not set"; }

std::unique_ptr<internal::CheckBase> Path::IsSet::clone() const { return std::make_unique<IsSet>(path_, name_); }

bool Path::Exists::valid() const { return std::filesystem::exists(path_); }

std::string Path::Exists::message() const { return "path '" + path_.string() + "' does not exist"; }

std::unique_ptr<internal::CheckBase> Path::Exists::clone() const { return std::make_unique<Exists>(path_, name_); }

bool Path::DoesNotExist::valid() const { return !std::filesystem::exists(path_); }

std::string Path::DoesNotExist::message() const { return "path '" + path_.string() + "' already exists"; }

std::unique_ptr<internal::CheckBase> Path::DoesNotExist::clone() const {
  return std::make_unique<DoesNotExist>(path_, name_);
}

bool Path::IsFile::valid() const { return std::filesystem::is_regular_file(path_); }

std::string Path::IsFile::message() const {
  if (!std::filesystem::exists(path_)) {
    return "path '" + path_.string() + "' does not exist";
  }
  return "path '" + path_.string() + "' is not a file";
}

std::unique_ptr<internal::CheckBase> Path::IsFile::clone() const { return std::make_unique<IsFile>(path_, name_); }

bool Path::IsDirectory::valid() const { return std::filesystem::is_directory(path_); }

std::string Path::IsDirectory::message() const {
  if (!std::filesystem::exists(path_)) {
    return "path '" + path_.string() + "' does not exist";
  }
  return "path '" + path_.string() + "' is not a directory";
}

std::unique_ptr<internal::CheckBase> Path::IsDirectory::clone() const {
  return std::make_unique<IsDirectory>(path_, name_);
}

bool Path::IsEmptyDirectory::valid() const {
  return std::filesystem::is_directory(path_) && std::filesystem::is_empty(path_);
}

std::string Path::IsEmptyDirectory::message() const {
  if (!std::filesystem::exists(path_)) {
    return "path '" + path_.string() + "' does not exist";
  }
  if (!std::filesystem::is_directory(path_)) {
    return "path '" + path_.string() + "' is not a directory";
  }
  return "path '" + path_.string() + "' is not an empty directory";
}

std::unique_ptr<internal::CheckBase> Path::IsEmptyDirectory::clone() const {
  return std::make_unique<IsEmptyDirectory>(path_, name_);
}

bool Path::HasExtension::valid() const {
  std::string extension = path_.extension().string();
  if (extension.empty()) {
    return false;
  }
  if (extension[0] == '.') {
    extension = extension.substr(1);
  }

  return extension == extension_;
}

std::string Path::HasExtension::message() const {
  std::string extension = path_.extension().string();
  if (extension.empty()) {
    return "path '" + path_.string() + "' is not a file with extension '" + extension_ + "' (has no extension)";
  }
  if (extension[0] == '.') {
    extension = extension.substr(1);
  }
  if (extension.empty()) {
    return "path '" + path_.string() + "' is not a file with extension '" + extension_ + "' (has no extension)";
  }
  return "path '" + path_.string() + "' is not a file with extension '" + extension_ + "' (is: '" + extension + "')";
}

std::unique_ptr<internal::CheckBase> Path::HasExtension::clone() const {
  return std::make_unique<HasExtension>(path_, extension_, name_);
}

}  // namespace config
