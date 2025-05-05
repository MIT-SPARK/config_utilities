# setup(
#     name=package_name,
#     version="0.0.0",
#     package_dir={"": "src"},
#     packages=find_packages("src"),
#     data_files=data_files,
#     install_requires=["setuptools"],
#     zip_safe=True,
#     maintainer="aaron",
#     maintainer_email="aaronray@mit.edu",
#     description="TODO: Package description",
#     license="TODO: License declaration",
#     tests_require=["pytest"],
#     entry_points={
#         "console_scripts": [
#             "spot_executor_node = spot_tools_ros.spot_executor_ros:main",
#         ],
#     },
# )

from setuptools import setup, find_packages

package_name = "config_utilities_ros"
setup(
    name=package_name,
    version="2.0.0",
    description="Tools for working with C++ config structs.",
    license="BSD-3-Clause",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Lukas Schmid",
    author_email="lschmid@mit.edu",
    url="https://github.com/MIT-SPARK/config_utilities",
)
