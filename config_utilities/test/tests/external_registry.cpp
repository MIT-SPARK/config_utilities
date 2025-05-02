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

#include <gtest/gtest.h>

#include "config_utilities/logging/log_to_stdout.h"
#include "config_utilities/settings.h"
#include "config_utilities/test/external_types.h"
#include "config_utilities/test/utils.h"

namespace config::test {

struct LoggerGuard {
  LoggerGuard() : logger(TestLogger::create()) {}
  ~LoggerGuard() { internal::Logger::setLogger(std::make_shared<internal::StdoutLogger>()); }
  std::shared_ptr<TestLogger> logger;
};

struct SettingsGuard {
  SettingsGuard() {}
  ~SettingsGuard() { Settings().restoreDefaults(); }
};

TEST(ExternalRegistry, MoveableGuard) {
  // guard is valid with at least one library
  internal::LibraryGuard guard("some_library_path");
  EXPECT_TRUE(guard);

  // default constructor is invalid state
  internal::LibraryGuard other = std::move(guard);
  EXPECT_FALSE(guard);
  EXPECT_TRUE(other);

  // move assignment should revert invalid/valid states
  guard = std::move(other);
  EXPECT_FALSE(other);
  EXPECT_TRUE(guard);
}

TEST(ExternalRegistry, InstanceLifetimes) {
  auto plugin_lib = loadExternalFactories("./test_config_utilities_plugins");
  auto unmanaged_logger = create<internal::Logger>("external_logger");
  EXPECT_TRUE(unmanaged_logger);
  unmanaged_logger.reset();

  auto internal_logger = createManaged(create<internal::Logger>("stdout"));
  auto external_logger = createManaged(create<internal::Logger>("external_logger"));
  ASSERT_TRUE(internal_logger);
  ASSERT_TRUE(external_logger);
  {  // limit scope for views
    const auto internal_view = internal_logger.get();
    EXPECT_TRUE(internal_view);
    const auto external_view = external_logger.get();
    EXPECT_TRUE(external_view);
  }

  // after unloading, we shouldn't be able to make a test logger
  plugin_lib.unload();
  unmanaged_logger = internal::ObjectFactory<internal::Logger>::create("external_logger");
  EXPECT_FALSE(unmanaged_logger);

  EXPECT_FALSE(internal_logger);
  EXPECT_FALSE(external_logger);
  {  // limit scope for views
    const auto internal_view = internal_logger.get();
    EXPECT_FALSE(internal_view);
    const auto external_view = external_logger.get();
    EXPECT_FALSE(external_view);
  }
}

TEST(ExternalRegistry, InvalidFile) {
  const LoggerGuard logger_guard;
  auto plugin_lib = loadExternalFactories("./some_invalid_file");
  EXPECT_FALSE(plugin_lib);
  ASSERT_GE(logger_guard.logger->numMessages(), 1);
  EXPECT_EQ(logger_guard.logger->lastMessage().find("Unable to load library"), 0);
}

TEST(ExternalRegistry, DisableLoading) {
  config::test::SettingsGuard settings_guard;
  config::Settings().external_libraries.enabled = false;

  const auto guard = config::loadExternalFactories("./test_config_utilities_plugins");
  EXPECT_FALSE(guard);
}

}  // namespace config::test

// globally namespaced to check example compilation
TEST(ExternalRegistry, ManagedInstance) {
  config::test::SettingsGuard settings_guard;
  config::Settings().external_libraries.log_allocation = true;

  config::ManagedInstance<config::test::Talker> talker;
  {  // scope where external library is loaded
    const std::vector<std::string> to_load{"./test_config_utilities_plugins"};
    const auto guard = config::loadExternalFactories(to_load);
    talker = config::createManaged(config::create<config::test::Talker>("external"));
    const auto view = talker.get();
    ASSERT_TRUE(view);
    EXPECT_EQ(view->talk(), "external");
  }  // external library is unloaded after this point

  const auto view = talker.get();
  EXPECT_FALSE(view);

  {  // scope where external library is loaded
    const std::vector<std::filesystem::path> to_load{"./test_config_utilities_plugins"};
    const auto guard = config::loadExternalFactories(to_load);
    talker = config::createManaged(config::create<config::test::Talker>("internal"));
    EXPECT_TRUE(talker);
  }  // external library is unloaded after this point

  // even internal types get unallocated
  EXPECT_FALSE(talker);
  const auto new_view = talker.get();
  EXPECT_FALSE(new_view);
}
