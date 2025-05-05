setup(
    name=package_name,
    version="0.0.0",
    package_dir={"": "src"},
    packages=find_packages("src"),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaron",
    maintainer_email="aaronray@mit.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spot_executor_node = spot_tools_ros.spot_executor_ros:main",
        ],
    },
)
