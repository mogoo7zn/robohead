from setuptools import find_packages, setup

package_name = "robogame_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
            ["launch/mission.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RoboGame 2026 Team",
    maintainer_email="robogame@example.com",
    description="Real-robot entry point: the mission node assembling the "
                "core stack around hardware backends.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_node = robogame_bringup.mission_node:main",
        ],
    },
)
