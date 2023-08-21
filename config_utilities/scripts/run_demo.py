#!/usr/bin/env python3
# ------------------------------------------------------------------------------
# Copyright (c) 2023, Massachusetts Institute of Technology.
# All Rights Reserved
#
# AUTHORS:     Lukas Schmid <lschmid@mit.edu>, Nathan Hughes <na26933@mit.edu>
# AFFILIATION: MIT-SPARK Lab, Massachusetts Institute of Technology
# YEAR:        2023
# LICENSE:     BSD 3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ------------------------------------------------------------------------------
"""Shim script for running one of the demo executables."""
import pathlib
import subprocess
import argparse
import sys


def _autodetect_build_path():
    script_path = pathlib.Path(__file__).absolute().parent
    ret = subprocess.run(
        ["catkin", "locate", "-b", "config_utilities"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=str(script_path),
        check=False,
    )
    if ret.returncode != 0:
        return None

    return pathlib.Path(ret.stdout.decode("utf-8").strip("\n"))


def _get_resource_path():
    project_dir = pathlib.Path(__file__).absolute().parent.parent
    return project_dir / "demos" / "resources"


def _verify_name(build_path, name):
    demo_path = build_path / "demos"
    executable_path = demo_path / f"demo_{name}"
    if not executable_path.exists():
        demos = [x for x in demo_path.glob("demo_*")]
        print("available demos:")
        for demo in demos:
            print(f"  - {demo.stem[5:]}")

        return None

    return executable_path


def main():
    """Run script."""
    parser = argparse.ArgumentParser(
        description="shim for running demo executables")
    parser.add_argument("name", help="executable name")
    parser.add_argument("--build_path",
                        "-b",
                        help="path to build directory",
                        nargs="?",
                        default=None)
    args = parser.parse_args()

    resource_path = _get_resource_path()
    print(f"using resource path: {resource_path}")

    if not args.build_path:
        build_path = _autodetect_build_path()
        if not build_path:
            print(
                "Unable to detect build path! Specify manually with -b/--build_path"
            )
            sys.exit(1)
    else:
        build_path = pathlib.Path(args.build_path)

    if not build_path.exists():
        print(f"{build_path} does not exist")
        sys.exit(1)

    executable_path = _verify_name(build_path, args.name)
    if not executable_path:
        print("invalid demo name")
        sys.exit(1)

    subprocess.run([str(executable_path), str(resource_path)], check=False)


if __name__ == "__main__":
    main()
