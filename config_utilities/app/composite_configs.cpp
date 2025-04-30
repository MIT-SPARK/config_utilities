#include <iostream>

#include <config_utilities/parsing/commandline.h>

const std::string help_msg =
    R"""(Usage: composite-config [--config-utilities-yaml YAML_TOKEN ...]... [--config-utilities-file FILEPATH[@NAMESPACE]]...

Merges the input YAML values from left to right and outputs the resulting composite YAML to stdout.
Invalid YAML or missing files get dropped during compositing.

Options:
  -h/--help: Show this message.
  --config-utilities-yaml: Takes an arbitrary set of tokens that form a valid YAML string.
                           Spaces are not required to be escaped, so `--config-utilities foo: value`
                           is parsed the same as `--config-utilities 'foo: value'`. Can be specified
                           multiple times.
  --config-utilities-file: Takes a filepath to YAML to load and composite. The YAML can optionally
                           be namespaced by `@NAMESPACE` where 'FILE@a/b' maps to
                           '{a: {b: FILE_CONTENTS}}'. Can be specified multiple times.

Example:
> echo "{a: 42, bar: hello}" > /tmp/test_in.yaml
> composite-configs --config-utilities-yaml "{foo: {bar: value, b: -1.0}}" --config-utilities-file /tmp/test_in.yaml@foo
{foo: {bar: hello, b: -1.0, a: 42}}

See https://github.com/MIT-SPARK/config_utilities/blob/main/docs/Parsing.md#parse-from-the-command-line
for more information.
)""";

int main(int argc, char* argv[]) {
  config::internal::ParserInfo info;
  auto result = config::internal::loadFromArguments(argc, argv, false, &info);
  if (info.help_present) {
    std::cerr << help_msg << std::endl;
    return 1;
  }

  switch (result.Type()) {
    case YAML::NodeType::Null:
    case YAML::NodeType::Undefined:
    default:
      break;
    case YAML::NodeType::Scalar:
    case YAML::NodeType::Sequence:
    case YAML::NodeType::Map:
      std::cout << result << std::endl;
      break;
  }

  return 0;
}
