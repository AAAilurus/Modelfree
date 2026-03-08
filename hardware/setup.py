import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'so100_hardware_bringup'

setup(
    name=package_name,
    version='0.1.0',
    # 'nodes' is the Python package that contains all node modules.
    # Using find_packages() discovers both 'nodes' (the subpackage in this
    # directory) and any other packages present.
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py'),
        ),
        # config files
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
        # data directory placeholder
        (
            os.path.join('share', package_name, 'data'),
            ['data/.gitkeep'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Real-hardware bringup for SO-100/SO-101 2-DOF leader-follower arms',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # These are the ROS2-executable node entry points.
            # After colcon build they appear in lib/so100_hardware_bringup/.
            'leader_hw_node = nodes.leader_hw_node:main',
            'follower_hw_node = nodes.follower_hw_node:main',
            'relay_node = nodes.relay_node:main',
            'csv_logger_node = nodes.csv_logger_node:main',
        ],
    },
)
