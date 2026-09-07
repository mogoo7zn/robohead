from setuptools import find_packages, setup

package_name = "robogame_adapters"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RoboGame 2026 Team",
    maintainer_email="robogame@example.com",
    description="Real-hardware backends for core "
                "(camera detectors over the same Protocols as the mocks).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={},
)
