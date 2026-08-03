from glob import glob

from setuptools import find_packages, setup

package_name = "drone_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        (
            "share/" + package_name + "/models/iris_with_gimbal_lidar",
            glob("models/iris_with_gimbal_lidar/*.*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="skylergrossi7-ops",
    maintainer_email="skylergrossi7-ops@users.noreply.github.com",
    description="Gazebo simulation assets for runner tracking.",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
        ],
    },
)
