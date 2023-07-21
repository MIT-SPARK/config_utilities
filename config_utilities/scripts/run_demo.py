#!/usr/bin/env python3
"""Shim script for running one of the demo executables."""
import pathlib
import subprocess
import argparse
import sys


def _autodetect_build_path():
    return None


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
    parser = argparse.ArgumentParser(description="shim for running demo executables")
    parser.add_argument("name", help="executable name")
    parser.add_argument(
        "--build_path", "-b", help="path to build directory", nargs="?", default=None
    )
    args = parser.parse_args()

    resource_path = _get_resource_path()
    print(f"using resource path: {resource_path}")

    if not args.build_path:
        build_path = _autodetect_build_path()
        if not build_path:
            print("Unable to detect build path! Specify manually with -b/--build_path")
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

    subprocess.run([str(executable_path), str(resource_path)])


if __name__ == "__main__":
    main()
