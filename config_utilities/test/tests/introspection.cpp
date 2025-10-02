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

#include "config_utilities/internal/introspection.h"

#include <gtest/gtest.h>

#include "config_utilities/config.h"
#include "config_utilities/factory.h"
#include "config_utilities/parsing/commandline.h"
#include "config_utilities/parsing/context.h"
#include "config_utilities/settings.h"
#include "config_utilities/test/cli_args.h"
#include "config_utilities/test/default_config.h"
#include "config_utilities/test/introspection_utils.h"
#include "config_utilities/test/utils.h"
#include "config_utilities/types/conversions.h"
#include "config_utilities/types/enum.h"
#include "config_utilities/virtual_config.h"

namespace config::test {

using Event = internal::Introspection::Event;
using By = internal::Introspection::By;
using Intro = internal::Introspection;

namespace {
bool with_namespace = false;
}

struct IntroBase {
  virtual ~IntroBase() = default;
};

struct IntroDerivedA : public IntroBase {
  struct Config {
    std::string s = "test";
  } const config;
  explicit IntroDerivedA(const Config& config) : config(config) {}
};

struct IntroDerivedB : public IntroBase {
  struct Config {
    std::string hi = "hello";
  } const config;
  explicit IntroDerivedB(const Config& config) : config(config) {}
};

struct IntroDerivedWithSubConfigs : public IntroBase {
  struct Config {
    config::VirtualConfig<IntroBase> defaulted{IntroDerivedA::Config()};
    config::VirtualConfig<IntroBase> non_defaulted;
  } const config;
  explicit IntroDerivedWithSubConfigs(const Config& config) : config(config) {}
};

struct IntroTestConfig {
  int a = 1;
  std::vector<float> vec = {1.0f, 2.0f, 3.0f};
  std::map<std::string, bool> map = {{"a", true}, {"b", false}};
  struct IntroSubConfig {
    double d = 3.1415;
  } subconfig;
  enum IntroEnum { ONE = 1, TWO = 2, THREE = 3 } e = IntroEnum::TWO;
  using VirtConf = VirtualConfig<IntroBase>;
  VirtConf unset_virtual;
  VirtConf virtual_config{IntroDerivedA::Config()};
  std::vector<VirtConf> vec_modules{VirtConf{IntroDerivedA::Config()}, VirtConf{IntroDerivedB::Config()}};
  std::map<std::string, VirtConf> map_modules{{"first", VirtConf{IntroDerivedA::Config()}},
                                              {"second", VirtConf{IntroDerivedB::Config()}}};
  std::vector<VirtConf> empty_vec_modules;
  std::map<std::string, VirtConf> empty_map_modules;
  std::vector<IntroSubConfig> sub_config_vec{IntroSubConfig(), IntroSubConfig()};
  std::vector<IntroSubConfig> empty_sub_config_vec;
  std::map<std::string, IntroSubConfig> sub_config_map{{"one", IntroSubConfig()}, {"two", IntroSubConfig()}};
  std::map<std::string, IntroSubConfig> empty_sub_config_map;
};

struct WrapperConfig {
  IntroTestConfig config;
};

void declare_config(IntroDerivedA::Config& config) {
  name("IntroDerivedA::Config");
  if (with_namespace) {
    enter_namespace("ns_for_A");
  }
  field(config.s, "s");
}

void declare_config(IntroDerivedB::Config& config) {
  name("IntroDerivedB::Config");
  field(config.hi, "hi");
}

void declare_config(IntroDerivedWithSubConfigs::Config& config) {
  name("IntroDerivedWithSubConfigs::Config");
  config.defaulted.setOptional();
  config.non_defaulted.setOptional();
  field(config.defaulted, "defaulted");
  field(config.non_defaulted, "non_defaulted");
}

void declare_config(IntroTestConfig::IntroSubConfig& config) {
  name("IntroSubConfig");
  if (with_namespace) {
    enter_namespace("sub_ns/subsub_ns");
  }
  field(config.d, "d");
}

void declare_config(IntroTestConfig& config) {
  name("IntroTestConfig");
  field(config.a, "a");
  field(config.vec, "vec");
  field(config.map, "map");
  field(config.subconfig, "subconfig");
  enum_field(config.e,
             "e",
             {{IntroTestConfig::IntroEnum::ONE, "ONE"},
              {IntroTestConfig::IntroEnum::TWO, "TWO"},
              {IntroTestConfig::IntroEnum::THREE, "THREE"}});
  config.unset_virtual.setOptional(true);
  field(config.unset_virtual, "unset_virtual");
  field(config.virtual_config, "virtual_config");
  field(config.vec_modules, "vec_modules");
  field(config.map_modules, "map_modules");
  field(config.empty_vec_modules, "empty_vec_modules");
  field(config.empty_map_modules, "empty_map_modules");
  if (with_namespace) {
    enter_namespace("vec_ns");
  }
  field(config.sub_config_vec, "sub_config_vec");
  field(config.empty_sub_config_vec, "empty_sub_config_vec");
  if (with_namespace) {
    switch_namespace("map_ns");
  }
  field(config.sub_config_map, "sub_config_map");
  field(config.empty_sub_config_map, "empty_sub_config_map");
}

void declare_config(WrapperConfig& config) {
  name("WrapperConfig");
  field(config.config, "config");
}

TEST(Introspection, invokeFromParser) {
  reset();
  CliArgs cli_args(std::vector<std::string>{"some_command",
                                            "--config-utilities-file",
                                            "resources/foo.yaml",
                                            "--config-utilities-file",
                                            "resources/bar.yaml",
                                            "--config-utilities-introspect",
                                            "path/to/output"});

  // With specified output directory
  auto args = cli_args.get();
  auto node = internal::loadFromArguments(args.argc, args.argv, true);
  EXPECT_EQ(args.get_cmd(), "some_command");
  EXPECT_EQ(config::Settings().introspection.output, "path/to/output");

  // Default output directory.
  cli_args = CliArgs(std::vector<std::string>{"some_command",
                                              "--config-utilities-file",
                                              "resources/foo.yaml",
                                              "--config-utilities-file",
                                              "resources/bar.yaml",
                                              "--config-utilities-introspect"});
  args = cli_args.get();
  node = internal::loadFromArguments(args.argc, args.argv, true);
  EXPECT_EQ(args.get_cmd(), "some_command");
  EXPECT_EQ(config::Settings().introspection.output, intro_dir);
}

TEST(Introspection, logCLIFile) {
  reset();
  CliArgs cli_args(std::vector<std::string>{"some_command", "--config-utilities-file", "resources/foo.yaml@foo", "-i"});
  auto args = cli_args.get();
  auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const std::string expected = R"(
foo:
  a: ['s1@f0:5']
  b:
    [0]: ['s1@f0:1']
    [1]: ['s1@f0:2']
    [2]: ['s1@f0:3']
  c: ['s1@f0:hello'])";
  EXPECT_EQ(Intro::instance().data().display(), expected);
}

TEST(Introspection, logCLIYaml) {
  reset();
  CliArgs cli_args(std::vector<std::string>{
      "some_command", "--config-utilities-yaml", "foo: {a: 5.0,  b: [1, 2, 3],  sub_ns: {c: hello}}", "-i"});
  auto args = cli_args.get();
  auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const std::string expected = R"(
foo:
  a: ['s1@a0:5']
  b:
    [0]: ['s1@a0:1']
    [1]: ['s1@a0:2']
    [2]: ['s1@a0:3']
  sub_ns:
    c: ['s1@a0:hello'])";
  EXPECT_EQ(Intro::instance().data().display(), expected);
}

TEST(Introspection, logCLISubstitution) {
  reset();
  // Set env variable.
  auto var = std::getenv("CONF_UTILS_RANDOM_ENV_VAR");
  if (var) {
    FAIL() << "environment variable 'CONF_UTILS_RANDOM_ENV_VAR' is already set.";
  }
  setenv("CONF_UTILS_RANDOM_ENV_VAR", "env_val", 1);

  // Parse.
  CliArgs cli_args(std::vector<std::string>{"some_command",
                                            "--config-utilities-yaml",
                                            "{val: 42, env: $<env | CONF_UTILS_RANDOM_ENV_VAR>, var: $<var | my_var>}",
                                            "-v",
                                            "my_var=var_val",
                                            "-i"});
  auto args = cli_args.get();
  auto node = internal::loadFromArguments(args.argc, args.argv, true);
  const std::string expected = R"(
val: ['s1@a0:42']
env: ['s1@a0:$<env | CONF_UTILS_RANDOM_ENV_VAR>', 'u2@s0:env_val']
var: ['s1@a0:$<var | my_var>', 'u2@s0:var_val'])";
  EXPECT_EQ(Intro::instance().data().display(), expected);
  unsetenv("CONF_UTILS_RANDOM_ENV_VAR");
}

TEST(Introspection, renderStateFromHistory) {
  reset();

  // Setup 1.
  YAML::Node node1 = YAML::Load("{a: 5, b: [1, 2], c: {d: 10, e: 20}}");
  Intro::logMerge(node1, node1, By::file("file0.yaml"));

  // Merge 2.
  YAML::Node in = YAML::Load("{a: 7, c: {e: {f: blipp}}}");
  YAML::Node node2 = YAML::Clone(node1);
  internal::mergeYamlNodes(node2, in, internal::MergeMode::APPEND);
  Intro::logMerge(node2, in, By::file("file1.yaml"));

  // Merge 3.
  in = YAML::Load("{b: [3], c: {d: 15}}");
  YAML::Node node3 = YAML::Clone(node2);
  internal::mergeYamlNodes(node3, in, internal::MergeMode::APPEND);
  Intro::logMerge(node3, in, By::file("file2.yaml"));

  // Overwriting sets: list to map & upstream scalars.
  in = YAML::Load("{b: {foo: bar}, c: 123}");
  YAML::Node node4 = YAML::Clone(node3);
  internal::mergeYamlNodes(node4, in, internal::MergeMode::REPLACE);
  Intro::logMerge(node4, in, By::file("file3.yaml"));

  // Clear.
  YAML::Node node5 = YAML::Node();
  Intro::logClear(By::programmatic("test clear"));

  // Set again with new values.
  in = YAML::Load("{x: 42, b: [{nested: true}, {nested: false}], c: {sub: \"<!emulate | subst>\"}}");
  YAML::Node node6 = YAML::Clone(node5);
  internal::mergeYamlNodes(node6, in, internal::MergeMode::APPEND);
  Intro::logMerge(node6, in, By::file("file4.yaml"));

  // Emulate substitution resolution.
  YAML::Node node7 = YAML::Clone(node6);
  node7["c"]["sub"] = "substituted_value";
  Intro::logDiff(node7, By::substitution("emulated substitution"));

  // Render out the history after the fact.
  auto rendered0 = Intro::instance().data().toYaml(0);
  YAML::Node empty;
  expectEqual(empty, rendered0);

  auto rendered1 = Intro::instance().data().toYaml(1);
  expectEqual(node1, rendered1);

  auto rendered2 = Intro::instance().data().toYaml(2);
  expectEqual(node2, rendered2);

  auto rendered3 = Intro::instance().data().toYaml(3);
  expectEqual(node3, rendered3);

  auto rendered4 = Intro::instance().data().toYaml(4);
  expectEqual(node4, rendered4);

  auto rendered5 = Intro::instance().data().toYaml(5);
  expectEqual(node5, rendered5);

  auto rendered6 = Intro::instance().data().toYaml(6);
  expectEqual(node6, rendered6);

  auto rendered7 = Intro::instance().data().toYaml(7);
  expectEqual(node7, rendered7);
}

TEST(Introspection, getValuesStructure) {
  reset();
  const auto regA = RegistrationGuard<IntroBase, IntroDerivedA, IntroDerivedA::Config>("IntroDerivedA");
  const auto regB = RegistrationGuard<IntroBase, IntroDerivedB, IntroDerivedB::Config>("IntroDerivedB");
  auto config = fromContext<IntroTestConfig>();
  const std::string expected = R"""(
a: ['a1@c0:1']
vec:
  [0]: ['a1@c0:1']
  [1]: ['a1@c0:2']
  [2]: ['a1@c0:3']
map:
  a: ['a1@c0:true']
  b: ['a1@c0:false']
e: ['a1@c0:TWO']
subconfig:
  d: ['a1@c1:3.1415']
unset_virtual:
  type: ['a1@c2:Uninitialized Virtual Config']
virtual_config:
  type: ['a1@c3:IntroDerivedA']
  s: ['a1@c3:test']
vec_modules:
  [0]:
    type: ['a1@c3:IntroDerivedA']
    s: ['a1@c3:test']
  [1]:
    type: ['a1@c4:IntroDerivedB']
    hi: ['a1@c4:hello']
map_modules:
  first:
    type: ['a1@c3:IntroDerivedA']
    s: ['a1@c3:test']
  second:
    type: ['a1@c4:IntroDerivedB']
    hi: ['a1@c4:hello']
sub_config_vec:
  [0]:
    d: ['a1@c1:3.1415']
  [1]:
    d: ['a1@c1:3.1415']
sub_config_map:
  one:
    d: ['a1@c1:3.1415']
  two:
    d: ['a1@c1:3.1415'])""";
  EXPECT_EQ(Intro::instance().data().display(), expected);
}

TEST(Introspection, getValuesNamespaced) {
  reset();
  const auto regA = RegistrationGuard<IntroBase, IntroDerivedA, IntroDerivedA::Config>("IntroDerivedA");
  const auto regB = RegistrationGuard<IntroBase, IntroDerivedB, IntroDerivedB::Config>("IntroDerivedB");
  with_namespace = true;
  auto config = fromContext<IntroTestConfig>("foo/bar");
  with_namespace = false;
  const std::string expected = R"""(
foo:
  bar:
    a: ['a1@c0:1']
    vec:
      [0]: ['a1@c0:1']
      [1]: ['a1@c0:2']
      [2]: ['a1@c0:3']
    map:
      a: ['a1@c0:true']
      b: ['a1@c0:false']
    e: ['a1@c0:TWO']
    subconfig:
      sub_ns:
        subsub_ns:
          d: ['a1@c1:3.1415']
    unset_virtual:
      type: ['a1@c2:Uninitialized Virtual Config']
    virtual_config:
      type: ['a1@c3:IntroDerivedA']
      ns_for_A:
        s: ['a1@c3:test']
    vec_modules:
      [0]:
        type: ['a1@c3:IntroDerivedA']
        ns_for_A:
          s: ['a1@c3:test']
      [1]:
        type: ['a1@c4:IntroDerivedB']
        hi: ['a1@c4:hello']
    map_modules:
      first:
        type: ['a1@c3:IntroDerivedA']
        ns_for_A:
          s: ['a1@c3:test']
      second:
        type: ['a1@c4:IntroDerivedB']
        hi: ['a1@c4:hello']
    vec_ns:
      sub_config_vec:
        [0]:
          sub_ns:
            subsub_ns:
              d: ['a1@c1:3.1415']
        [1]:
          sub_ns:
            subsub_ns:
              d: ['a1@c1:3.1415']
    map_ns:
      sub_config_map:
        one:
          sub_ns:
            subsub_ns:
              d: ['a1@c1:3.1415']
        two:
          sub_ns:
            subsub_ns:
              d: ['a1@c1:3.1415'])""";
  EXPECT_EQ(Intro::instance().data().display(), expected);
}

TEST(Introspection, getValues) {
  clearContext();
  reset();
  const auto regA = RegistrationGuard<IntroBase, IntroDerivedA, IntroDerivedA::Config>("IntroDerivedA");
  const auto regB = RegistrationGuard<IntroBase, IntroDerivedB, IntroDerivedB::Config>("IntroDerivedB");
  auto context_data = YAML::Load(R"(
a: 123 # get
vec: [1, 2, 3] # default
map: {a: false} # get, absent 
e: NONEXISTENT # fail parsing
empty_vec_modules: [ {type: IntroDerivedA, s: overridden}, {type: IntroDerivedB} ] # create vec modules
)");
  // NOTE(lschmid): The type params are always recognised as default values as the default checks happen on a per-struct
  // level and each moule canonly have their own type.
  pushToContext(context_data);
  auto config = fromContext<IntroTestConfig>();
  const std::string expected = R"""(
a: ['s1@p0:123', 'g2@c0:123']
vec:
  [0]: ['s1@p0:1', 'd2@c0:1']
  [1]: ['s1@p0:2', 'd2@c0:2']
  [2]: ['s1@p0:3', 'd2@c0:3']
map:
  a: ['s1@p0:false', 'g2@c0:false']
e: ['s1@p0:NONEXISTENT', 'e2@c0:TWO']
empty_vec_modules:
  [0]:
    type: ['s1@p0:IntroDerivedA', 'g2@c3:IntroDerivedA']
    s: ['s1@p0:overridden', 'g2@c3:overridden']
  [1]:
    type: ['s1@p0:IntroDerivedB', 'g2@c4:IntroDerivedB']
    hi: ['a2@c4:hello']
subconfig:
  d: ['a2@c1:3.1415']
unset_virtual:
  type: ['a2@c2:Uninitialized Virtual Config']
virtual_config:
  type: ['a2@c3:IntroDerivedA']
  s: ['a2@c3:test']
vec_modules:
  [0]:
    type: ['a2@c3:IntroDerivedA']
    s: ['a2@c3:test']
  [1]:
    type: ['a2@c4:IntroDerivedB']
    hi: ['a2@c4:hello']
map_modules:
  first:
    type: ['a2@c3:IntroDerivedA']
    s: ['a2@c3:test']
  second:
    type: ['a2@c4:IntroDerivedB']
    hi: ['a2@c4:hello']
sub_config_vec:
  [0]:
    d: ['a2@c1:3.1415']
  [1]:
    d: ['a2@c1:3.1415']
sub_config_map:
  one:
    d: ['a2@c1:3.1415']
  two:
    d: ['a2@c1:3.1415'])""";
  EXPECT_EQ(Intro::instance().data().display(), expected);
}

TEST(Introspection, create) {
  reset();
  clearContext();
  const auto regA = RegistrationGuard<IntroBase, IntroDerivedA, IntroDerivedA::Config>("IntroDerivedA");

  // No factory param.
  auto obj = createFromContext<IntroBase>();
  EXPECT_FALSE(obj);
  std::string expected = R"(
type: ['e1@c0'])";
  EXPECT_EQ(Intro::instance().data().display(), expected);

  // Get object at namespace.
  pushToContext(YAML::Load("ns: {nns: {type: IntroDerivedA, s: custom}}"));
  obj = createFromContextWithNamespace<IntroBase>("ns/nns");
  EXPECT_TRUE(obj);
  expected = R"(
type: ['e1@c0']
ns:
  nns:
    type: ['s3@p1:IntroDerivedA', 'g4@c1:IntroDerivedA']
    s: ['s3@p1:custom', 'g4@c1:custom'])";
  EXPECT_EQ(Intro::instance().data().display(), expected);
}

TEST(Introspection, setConfigSettingsFromContext) {
  reset();
  clearContext();
  pushToContext(YAML::Load(R"(
global_settings:
  printing:
    width: 123
  external_libraries:
    enabled: true)"));
  setConfigSettingsFromContext("global_settings");
  const std::string expected = R"(
global_settings:
  printing:
    width: ['s2@p1:123', 'g3@c1:123']
    indent: ['a3@c1:30']
    subconfig_indent: ['a3@c1:3']
    show_defaults: ['a3@c1:true']
    show_units: ['a3@c1:true']
    inline_subconfigs: ['a3@c1:true']
    reformat_floats: ['a3@c1:true']
    show_missing: ['a3@c1:false']
    show_subconfig_types: ['a3@c1:true']
    show_virtual_configs: ['a3@c1:true']
    show_num_checks: ['a3@c1:true']
    print_meta_fields: ['a3@c1:false']
  external_libraries:
    enabled: ['s2@p1:true', 'd3@c3:true']
    verbose_load: ['a3@c3:true']
    log_allocation: ['a3@c3:false']
  disable_default_stdout_logger: ['a3@c0:false']
  factory:
    type_param_name: ['a3@c2:type']
  introspection:
    output: ['a3@c4:config_introspection_output'])";
  EXPECT_EQ(Intro::instance().data().display(), expected);
  Settings().restoreDefaults();
}

TEST(Introspection, identifyDefaultVirtualConfigs) {
  reset();
  clearContext();
  const auto regA = RegistrationGuard<IntroBase, IntroDerivedA, IntroDerivedA::Config>("IntroDerivedA");
  const auto regB = RegistrationGuard<IntroBase, IntroDerivedB, IntroDerivedB::Config>("IntroDerivedB");
  const auto regC = RegistrationGuard<IntroBase, IntroDerivedWithSubConfigs, IntroDerivedWithSubConfigs::Config>(
      "IntroDerivedWithSubConfigs");

  // Set values for new virtual configs, some default some others not.
  pushToContext(YAML::Load(R"(
config:
  unset_virtual: {type: IntroDerivedA}
  virtual_config: {type: IntroDerivedB}
  vec_modules: [ {type: IntroDerivedA}, {type: IntroDerivedA} ]
  map_modules: {first: {type: IntroDerivedA}, second: {type: IntroDerivedA} }
  empty_vec_modules: [ {type: IntroDerivedA}, {type: IntroDerivedB} ]
  empty_map_modules: {first: {type: IntroDerivedA}, second: {type: IntroDerivedA} })"));
  auto config = fromContext<WrapperConfig>();
  std::string expected = R"(
config:
  unset_virtual:
    type: ['s2@p1:IntroDerivedA', 'g3@c3:IntroDerivedA']
    s: ['a3@c3:test']
  virtual_config:
    type: ['s2@p1:IntroDerivedB', 'g3@c4:IntroDerivedB']
    hi: ['a3@c4:hello']
  vec_modules:
    [0]:
      type: ['s2@p1:IntroDerivedA', 'd3@c3:IntroDerivedA']
      s: ['a3@c3:test']
    [1]:
      type: ['s2@p1:IntroDerivedA', 'g3@c3:IntroDerivedA']
      s: ['a3@c3:test']
  map_modules:
    first:
      type: ['s2@p1:IntroDerivedA', 'd3@c3:IntroDerivedA']
      s: ['a3@c3:test']
    second:
      type: ['s2@p1:IntroDerivedA', 'g3@c3:IntroDerivedA']
      s: ['a3@c3:test']
  empty_vec_modules:
    [0]:
      type: ['s2@p1:IntroDerivedA', 'g3@c3:IntroDerivedA']
      s: ['a3@c3:test']
    [1]:
      type: ['s2@p1:IntroDerivedB', 'g3@c4:IntroDerivedB']
      hi: ['a3@c4:hello']
  empty_map_modules:
    first:
      type: ['s2@p1:IntroDerivedA', 'g3@c3:IntroDerivedA']
      s: ['a3@c3:test']
    second:
      type: ['s2@p1:IntroDerivedA', 'g3@c3:IntroDerivedA']
      s: ['a3@c3:test']
  a: ['a3@c1:1']
  vec:
    [0]: ['a3@c1:1']
    [1]: ['a3@c1:2']
    [2]: ['a3@c1:3']
  map:
    a: ['a3@c1:true']
    b: ['a3@c1:false']
  e: ['a3@c1:TWO']
  subconfig:
    d: ['a3@c2:3.1415']
  sub_config_vec:
    [0]:
      d: ['a3@c2:3.1415']
    [1]:
      d: ['a3@c2:3.1415']
  sub_config_map:
    one:
      d: ['a3@c2:3.1415']
    two:
      d: ['a3@c2:3.1415'])";
  EXPECT_EQ(Intro::instance().data().display(), expected);

  clearContext();
  reset();
  pushToContext(YAML::Load(R"(
defaulted:
  type: IntroDerivedWithSubConfigs
non_defaulted: 
  type: IntroDerivedWithSubConfigs
  defaulted:
    type: IntroDerivedWithSubConfigs
    defaulted: {type: IntroDerivedA}
    non_defaulted: {type: IntroDerivedB})"));

  auto config2 = fromContext<IntroDerivedWithSubConfigs::Config>();
  expected = R"(
defaulted:
  type: ['s1@p0:IntroDerivedWithSubConfigs', 'g2@c0:IntroDerivedWithSubConfigs']
  defaulted:
    type: ['a2@c1:IntroDerivedA']
    s: ['a2@c1:test']
  non_defaulted:
    type: ['a2@c2:Uninitialized Virtual Config']
non_defaulted:
  type: ['s1@p0:IntroDerivedWithSubConfigs', 'g2@c0:IntroDerivedWithSubConfigs']
  defaulted:
    type: ['s1@p0:IntroDerivedWithSubConfigs', 'g2@c0:IntroDerivedWithSubConfigs']
    defaulted:
      type: ['s1@p0:IntroDerivedA', 'd2@c1:IntroDerivedA']
      s: ['a2@c1:test']
    non_defaulted:
      type: ['s1@p0:IntroDerivedB', 'g2@c3:IntroDerivedB']
      hi: ['a2@c3:hello']
  non_defaulted:
    type: ['a2@c2:Uninitialized Virtual Config'])";
  EXPECT_EQ(Intro::instance().data().display(), expected);
  disable();  // Clean up after last test.
}

}  // namespace config::test
