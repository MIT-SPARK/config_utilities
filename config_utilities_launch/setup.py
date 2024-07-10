from setuptools import find_packages, setup

package_name = "config_utilities_launch"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nathan",
    maintainer_email="nathan.h.hughes@gmail.com",
    description="TODO: Package description",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "launch.frontend.launch_extension": [
            "config_utilities_launch = config_utilities_launch"
        ],
    },
)
