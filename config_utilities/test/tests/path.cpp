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

#include <fstream>

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/printing.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/validation.h"

namespace config::test {

struct TmpDir {
  TmpDir() {
    if (std::filesystem::exists(path_)) {
      std::filesystem::remove_all(path_);
    } else {
      std::filesystem::create_directories(path_);
    }
  }
  ~TmpDir() {
    std::filesystem::remove_all(path_);
    std::filesystem::remove(path_);
  }
  const std::string path() const { return path_.string(); }

 private:
  const std::filesystem::path path_ = std::filesystem::temp_directory_path() / "config_utilities_test";
};

struct PathConfig {
  std::string str_path;
  std::filesystem::path path;
};

void declare_config(PathConfig& config) {
  name("PathConfig");
  field<Path>(config.str_path, "str_path");
  field<Path>(config.path, "path");
  check<Path::HasExtension>(config.str_path, ".txt", "str_path");
  check<Path::Exists>(config.path, "path");
}

TEST(Path, Conversion) {
  const std::string expected = "/tmp/a/test.txt";
  const std::string with_dot = "/tmp/a/./test.txt";
  const std::string with_dotdot = "/tmp/a/../a/test.txt";
  const std::string with_slashes = "/tmp//////a/test.txt";
  const std::filesystem::path expected_path(expected);
  const std::filesystem::path with_dot_path(with_dot);
  const std::filesystem::path with_dotdot_path(with_dotdot);
  const std::filesystem::path with_slashes_path(with_slashes);
  std::string error;

  EXPECT_EQ(Path::toIntermediate(expected, error), expected);
  EXPECT_EQ(Path::toIntermediate(with_dot, error), expected);
  EXPECT_EQ(Path::toIntermediate(with_dotdot, error), expected);
  EXPECT_EQ(Path::toIntermediate(with_slashes, error), expected);
  EXPECT_EQ(Path::toIntermediate(expected_path, error), expected);
  EXPECT_EQ(Path::toIntermediate(with_dot_path, error), expected);
  EXPECT_EQ(Path::toIntermediate(with_dotdot_path, error), expected);
  EXPECT_EQ(Path::toIntermediate(with_slashes_path, error), expected);

  std::filesystem::path path;
  std::string str;
  Path::fromIntermediate(expected, path, error);
  EXPECT_EQ(path, expected_path);
  Path::fromIntermediate(expected, str, error);
  EXPECT_EQ(str, expected);

  path.clear();
  str.clear();
  Path::fromIntermediate(with_dot, path, error);
  EXPECT_EQ(path, expected_path);
  Path::fromIntermediate(with_dot, str, error);
  EXPECT_EQ(str, expected);

  path.clear();
  str.clear();
  Path::fromIntermediate(with_dotdot, path, error);
  EXPECT_EQ(path, expected_path);
  Path::fromIntermediate(with_dotdot, str, error);
  EXPECT_EQ(str, expected);

  path.clear();
  str.clear();
  Path::fromIntermediate(with_slashes, path, error);
  EXPECT_EQ(path, expected_path);
  Path::fromIntermediate(with_slashes, str, error);
  EXPECT_EQ(str, expected);
}

TEST(Path, IsSet) {
  const std::filesystem::path empty_path = "";
  const std::filesystem::path non_empty_path = "/tmp/a/test.txt";
  const std::string empty_str = "";
  const std::string non_empty_str = "/tmp/a/test.txt";

  Path::IsSet empty_path_check(empty_path, "path");
  EXPECT_FALSE(empty_path_check.valid());
  EXPECT_EQ(empty_path_check.message(), "path is not set");

  Path::IsSet non_empty_path_check(non_empty_path, "path");
  EXPECT_TRUE(non_empty_path_check.valid());

  Path::IsSet empty_str_check(empty_str, "path");
  EXPECT_FALSE(empty_str_check.valid());
  EXPECT_EQ(empty_str_check.message(), "path is not set");

  Path::IsSet non_empty_str_check(non_empty_str, "path");
  EXPECT_TRUE(non_empty_str_check.valid());
}

TEST(Path, Exists) {
  TmpDir dir;
  std::filesystem::create_directories(dir.path() + "/tmp_dir");
  std::ofstream(dir.path() + "/tmp_file.txt").close();

  Path::Exists good_dir_check(dir.path() + "/tmp_dir", "path");
  EXPECT_TRUE(good_dir_check.valid());

  Path::Exists bad_dir_check(dir.path() + "/tmp_dir2", "path");
  EXPECT_FALSE(bad_dir_check.valid());
  EXPECT_EQ(bad_dir_check.message(), "path '" + dir.path() + "/tmp_dir2' does not exist");

  Path::Exists good_file_check(dir.path() + "/tmp_file.txt", "path");
  EXPECT_TRUE(good_file_check.valid());

  Path::Exists bad_file_check(dir.path() + "/tmp_file2.txt", "path");
  EXPECT_FALSE(bad_file_check.valid());
  EXPECT_EQ(bad_file_check.message(), "path '" + dir.path() + "/tmp_file2.txt' does not exist");
}

TEST(Path, DoesNotExist) {
  TmpDir dir;
  std::filesystem::create_directories(dir.path() + "/tmp_dir");
  std::ofstream(dir.path() + "/tmp_file.txt").close();

  Path::DoesNotExist good_dir_check(dir.path() + "/tmp_dir2", "path");
  EXPECT_TRUE(good_dir_check.valid());

  Path::DoesNotExist bad_dir_check(dir.path() + "/tmp_dir", "path");
  EXPECT_FALSE(bad_dir_check.valid());
  EXPECT_EQ(bad_dir_check.message(), "path '" + dir.path() + "/tmp_dir' already exists");

  Path::DoesNotExist good_file_check(dir.path() + "/tmp_file2.txt", "path");
  EXPECT_TRUE(good_file_check.valid());

  Path::DoesNotExist bad_file_check(dir.path() + "/tmp_file.txt", "path");
  EXPECT_FALSE(bad_file_check.valid());
  EXPECT_EQ(bad_file_check.message(), "path '" + dir.path() + "/tmp_file.txt' already exists");
}

TEST(Path, IsFile) {
  TmpDir dir;
  std::filesystem::create_directories(dir.path() + "/tmp_dir");
  std::ofstream(dir.path() + "/tmp_file.txt").close();

  Path::IsFile good_file_check(dir.path() + "/tmp_file.txt", "path");
  EXPECT_TRUE(good_file_check.valid());

  Path::IsFile bad_file_check(dir.path() + "/tmp_dir", "path");
  EXPECT_FALSE(bad_file_check.valid());
  EXPECT_EQ(bad_file_check.message(), "path '" + dir.path() + "/tmp_dir' is not a file");

  Path::IsFile non_existent_file_check(dir.path() + "/tmp_file2.txt", "path");
  EXPECT_FALSE(non_existent_file_check.valid());
  EXPECT_EQ(non_existent_file_check.message(), "path '" + dir.path() + "/tmp_file2.txt' does not exist");
}

TEST(Path, IsDirectory) {
  TmpDir dir;
  std::filesystem::create_directories(dir.path() + "/tmp_dir");
  std::ofstream(dir.path() + "/tmp_file.txt").close();

  Path::IsDirectory good_dir_check(dir.path() + "/tmp_dir", "path");
  EXPECT_TRUE(good_dir_check.valid());

  Path::IsDirectory bad_dir_check(dir.path() + "/tmp_file.txt", "path");
  EXPECT_FALSE(bad_dir_check.valid());
  EXPECT_EQ(bad_dir_check.message(), "path '" + dir.path() + "/tmp_file.txt' is not a directory");

  Path::IsDirectory non_existent_dir_check(dir.path() + "/tmp_dir2", "path");
  EXPECT_FALSE(non_existent_dir_check.valid());
  EXPECT_EQ(non_existent_dir_check.message(), "path '" + dir.path() + "/tmp_dir2' does not exist");
}

TEST(Path, IsEmptyDirectory) {
  TmpDir dir;
  std::filesystem::create_directories(dir.path() + "/tmp_dir");
  std::ofstream(dir.path() + "/tmp_file.txt").close();

  Path::IsEmptyDirectory good_dir_check(dir.path() + "/tmp_dir", "path");
  EXPECT_TRUE(good_dir_check.valid());

  Path::IsEmptyDirectory bad_dir_check(dir.path(), "path");
  EXPECT_FALSE(bad_dir_check.valid());
  EXPECT_EQ(bad_dir_check.message(), "path '" + dir.path() + "' is not an empty directory");

  Path::IsEmptyDirectory non_existent_dir_check(dir.path() + "/tmp_dir2", "path");
  EXPECT_FALSE(non_existent_dir_check.valid());
  EXPECT_EQ(non_existent_dir_check.message(), "path '" + dir.path() + "/tmp_dir2' does not exist");

  Path::IsEmptyDirectory not_a_dir_check(dir.path() + "/tmp_file.txt", "path");
  EXPECT_FALSE(not_a_dir_check.valid());
  EXPECT_EQ(not_a_dir_check.message(), "path '" + dir.path() + "/tmp_file.txt' is not a directory");
}

TEST(Path, HasExtension) {
  const std::string txt_path = "/tmp/test.txt";
  const std::string csv_path = "/tmp/test.csv";
  const std::string no_ext_path = "/tmp/test";
  const std::string no_ext_path2 = "/tmp/test.";
  const std::string no_ext_path3 = "/tmp/test/";

  Path::HasExtension with_dot_check(txt_path, ".txt", "path");
  EXPECT_TRUE(with_dot_check.valid());

  Path::HasExtension without_dot_check(csv_path, "csv", "path");
  EXPECT_TRUE(without_dot_check.valid());

  Path::HasExtension bad_ext_check(txt_path, "csv", "path");
  EXPECT_FALSE(bad_ext_check.valid());
  EXPECT_EQ(bad_ext_check.message(), "path '" + txt_path + "' is not a file with extension 'csv' (is: 'txt')");

  Path::HasExtension no_ext_check(no_ext_path, "txt", "path");
  EXPECT_FALSE(no_ext_check.valid());
  EXPECT_EQ(no_ext_check.message(), "path '" + no_ext_path + "' is not a file with extension 'txt' (has no extension)");

  Path::HasExtension no_ext_check2(no_ext_path2, "txt", "path");
  EXPECT_FALSE(no_ext_check2.valid());
  EXPECT_EQ(no_ext_check2.message(),
            "path '" + no_ext_path2 + "' is not a file with extension 'txt' (has no extension)");

  Path::HasExtension no_ext_check3(no_ext_path3, "txt", "path");
  EXPECT_FALSE(no_ext_check3.valid());
  EXPECT_EQ(no_ext_check3.message(),
            "path '" + no_ext_path3 + "' is not a file with extension 'txt' (has no extension)");
}

TEST(Path, Config) {
  TmpDir dir;

  const std::string yaml = "{str_path: " + dir.path() + "/test.csv, path: " + dir.path() + "/test.txt}";
  YAML::Node node = YAML::Load(yaml);
  const std::string expected_str = dir.path() + "/test.csv";
  const std::filesystem::path expected_path(dir.path() + "/test.txt");

  // Set values.
  PathConfig config;
  internal::Visitor::setValues(config, node);
  EXPECT_EQ(config.str_path, expected_str);
  EXPECT_EQ(config.path, expected_path);

  // Fail checks.
  EXPECT_FALSE(isValid(config));
  internal::MetaData meta_data = internal::Visitor::getChecks(config);
  ASSERT_EQ(meta_data.checks.size(), 2);
  EXPECT_EQ(meta_data.checks[0]->name(), "str_path");
  EXPECT_FALSE(meta_data.checks[0]->valid());
  EXPECT_EQ(meta_data.checks[1]->name(), "path");
  EXPECT_FALSE(meta_data.checks[1]->valid());

  // Pass checks.
  config.str_path = dir.path() + "/test.txt";
  std::ofstream(config.path).close();
  meta_data = internal::Visitor::getChecks(config);
  EXPECT_TRUE(isValid(config, true));
  ASSERT_EQ(meta_data.checks.size(), 2);
  EXPECT_TRUE(meta_data.checks[0]->valid());
  EXPECT_TRUE(meta_data.checks[1]->valid());

  // Get values.
  meta_data = internal::Visitor::getValues(config);
  node["str_path"] = dir.path() + "/test.txt";
  node["path"] = dir.path() + "/test.txt";
  expectEqual(meta_data.data, node);
}

}  // namespace config::test
