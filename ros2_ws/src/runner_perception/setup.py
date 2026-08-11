from setuptools import find_packages, setup

package_name = 'runner_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skyler',
    maintainer_email='skyler@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        "perception_node = runner_perception.perception_node:main",
        "camera_info_node = runner_perception.camera_info_node:main",
        "depth_anything_node = runner_perception.depth_anything_node:main",
        "yolo_masking_node = runner_perception.yolo_masking_node:main",
        ],
    },
)
