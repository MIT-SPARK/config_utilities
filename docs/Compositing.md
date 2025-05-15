# Compositing multiple YAML sources

**Contents:**
- [Compositing](#compositing)
- [How compositing works](#how-compositing-works)
- [Controlling compositing behavior](#controlling-compositing-behavior)
- [Substitutions](#substitutions)

## Compositing

`config_utilities` includes a command line tool that will take multiple sources of YAML and dump the combined (composited) YAML to `stdout`. This is designed for the use-case where a base YAML file contains the majority of a configuration, and small overlays are used to adapt or fill in the configuration for different variants of a system.

You can run `composite-configs -h` for more information on the utility.

## How compositing works

This tutorial explains how different sources of YAML data are combined into a single YAML tree, and is primarily a more in-depth explanation of the behavior of the command line based YAML parser.
Functionally, the command line parser is doing the following after parsing the various command line options.

```cpp
std::vector<YAML::Node> inputs; // parsed YAML from either files or raw YAML strings

YAML::Node combined; // result YAML node
for (const auto& input : inputs) {
  internal::mergeYamlNodes(combined, input, MergeMode::APPEND);
}
```

## Controlling compositing behavior

The default ROS-like merging behavior (append) can be overridden by inline tags. The following behaviors are currently available:
  - `!append`: Matched sequences are appended together (specifically, the right sequence is appended to the left)
  - `!replace`: Matched keys result in the right key overriding the left
  - `!merge`: Matched keys (including sequence indices) are recursed into. Any unmatched keys are added

These merging behaviors apply to all children below the tag (until another tag is present).

Example behavior:
```yaml
# original YAML (left)
root: {child: {a: 42, c: 0}, numbers: [1, 2, 3], scalar: -1}
# new YAML to merge (right)
root: !TAG {child: {a: 12, b: 13}, numbers: [4, 5], other: temp}

# result of merging right into left with !append in place of !TAG
root: {child: {a: 12, c: 0, b: 13}, numbers: [1, 2, 3, 4, 5], scalar: -1, other: temp}
# result of merging right into left with !replace in place of !TAG
root: {child: {a: 12, b: 13}, numbers: [4, 5], scalar: -1, other: temp}
# result of merging right into left with !merge in place of !TAG
root: {child: {a: 12, c: 0, b: 13}, numbers: [4, 5, 3], scalar: -1, other: temp}
```

## Substitutions

We also support a substitution language for interpolating values into YAML data.
These (closely) resemble the ROS substitution language.
Substitutions are performed with respect to a context that may have some variables that can be used.

As an example, resolving substitutions in the following YAML (with the context `num_prints = 5`)
```yaml
input_filepath: $<env HOME>/some/random/file
num_prints: $<var num_prints>
```
would result in
```yaml
input_filepath: /home/user/some/random/file
num_prints: 5
```
after calling `resolveSubstitutions` on the parsed YAML.
