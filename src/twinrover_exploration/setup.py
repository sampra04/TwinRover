import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'twinrover_exploration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='me597',
    maintainer_email='samarth.prashanth@gmail.com',
    description='TwinRover exploration stack for autonomous frontier mapping.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = twinrover_exploration.frontier_explorer:main',
            'map_saver_periodic = twinrover_exploration.map_saver_periodic:main',
            'status_monitor = twinrover_exploration.status_monitor:main',
            'teleop_override = twinrover_exploration.teleop_override:main',
        ],
    },
)
