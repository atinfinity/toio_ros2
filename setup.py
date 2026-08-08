from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'toio_ros2'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atinfinity',
    maintainer_email='dandelion1124@gmail.com',
    description='toio_ros2',
    license='Apache-2.0',
    # colcon needs a declared pytest dependency to run tests with pytest
    # instead of unittest (which collects nothing here)
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'toio_ros2_node = toio_ros2.toio_ros2_node:main',
        ],
    },
)
