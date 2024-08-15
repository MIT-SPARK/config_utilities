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

#include <config_utilities/internal/visitor.h>
#include <gtest/gtest.h>

#include "config_utilities/test/default_config.h"
#include "config_utilities/test/utils.h"

namespace config::test {

TEST(FieldInputInfo, GetInfo) {
  DefaultConfig config;
  const internal::MetaData data = internal::Visitor::getInfo(config);
  auto info = data.serializeFieldInfos();
  const std::string expected = R"(
type: config
name: DefaultConfig
fields:
  - type: field
    name: i
    unit: m
    value: 1
    default: 1
    input_info:
      type: int
      min: -2147483648
      max: 2147483647
  - type: field
    name: f
    unit: s
    value: 2.1
    default: 2.1
    input_info:
      type: float
      min: -inf
      max: inf
  - type: field
    name: d
    unit: m/s
    value: 3.2
    default: 3.2
    input_info:
      type: float
      min: -inf
      max: inf
  - type: field
    name: b
    value: true
    default: true
    input_info:
      type: bool
  - type: field
    name: u8
    value: 4
    default: 4
    input_info:
      type: int
      min: 0
      max: 255
  - type: field
    name: s
    value: test string
    default: test string
    input_info:
      type: string
  - type: field
    name: vec
    unit: frames
    value:
      - 1
      - 2
      - 3
    default:
      - 1
      - 2
      - 3
    input_info:
      type: yaml
  - type: field
    name: map
    value:
      a: 1
      b: 2
      c: 3
    default:
      a: 1
      b: 2
      c: 3
    input_info:
      type: yaml
  - type: field
    name: set
    value:
      - 1.1
      - 2.2
      - 3.3
    default:
      - 1.1
      - 2.2
      - 3.3
    input_info:
      type: yaml
  - type: field
    name: mat
    value:
      -
        - 1
        - 0
        - 0
      -
        - 0
        - 1
        - 0
      -
        - 0
        - 0
        - 1
    default:
      -
        - 1
        - 0
        - 0
      -
        - 0
        - 1
        - 0
      -
        - 0
        - 0
        - 1
    input_info:
      type: yaml
  - type: field
    name: my_enum
    value: A
    default: A
    input_info:
      type: options
      options:
        - A
        - B
        - C
  - type: field
    name: my_strange_enum
    value: X
    default: X
    input_info:
      type: options
      options:
        - Z
        - X
        - Y
  - type: config
    name: SubConfig
    field_name: sub_config
    fields:
      - type: field
        name: i
        value: 1
        default: 1
        input_info:
          type: int
          min: -2147483648
          max: 2147483647
      - type: config
        name: SubSubConfig
        field_name: sub_sub_config
        fields:
          - type: field
            name: i
            value: 1
            default: 1
            input_info:
              type: int
              min: -2147483648
              max: 2147483647
  - type: config
    name: SubSubConfig
    field_name: sub_sub_config
    fields:
      - type: field
        name: i
        value: 1
        default: 1
        input_info:
          type: int
          min: -2147483648
          max: 2147483647
)";
  // expectEqual(info, YAML::Load(expected));

  // std::cout << info << std::endl;
}

}  // namespace config::test
