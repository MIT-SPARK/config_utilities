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

namespace config::test {

TEST(ExternalRegistry, InstanceLifetimes) {
  auto plugin_lib = loadExternalFactories("./test_config_utilities_plugins");

  auto test_logger = create<internal::Logger>("test_logger");
  EXPECT_TRUE(test_logger);
  test_logger.reset();

  auto internal_logger = internal::ExternalRegistry::createManaged(create<internal::Logger>("stdout"));
  auto external_logger = internal::ExternalRegistry::createManaged(create<internal::Logger>("test_logger"));
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
  test_logger = internal::ObjectFactory<internal::Logger>::create("test_logger");
  EXPECT_FALSE(test_logger);

  EXPECT_TRUE(internal_logger);
  EXPECT_FALSE(external_logger);
  {  // limit scope for views
    const auto internal_view = internal_logger.get();
    EXPECT_TRUE(internal_view);
    const auto external_view = external_logger.get();
    EXPECT_FALSE(external_view);
  }
}

}  // namespace config::test
