from glob import glob

from setuptools import find_packages, setup


package_name = "drone_control"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="skylergrossi7-ops",
    maintainer_email="skylergrossi7-ops@users.noreply.github.com",
    description="Safe ROS 2 runner-following control for the simulated drone.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "runner_follower = drone_control.runner_follower_node:main",
        ],
    },
)
