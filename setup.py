from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tus',
    maintainer_email='tus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "fictitious_force_node = robot_controller.fictitious_force_node:main",
            "slave_controller = robot_controller.slave_controller:main",
            "data_logger = robot_controller.data_logger:main",
            "data_logger_realtime = robot_controller.data_logger_realtime:main",
        ],
    },
)
