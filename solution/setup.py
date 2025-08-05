import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@york.ac.uk',
    description='AURO Assessment Solution',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = solution.robot_controller:main',
            'data_logger = solution.data_logger:main',
        ],
    },
)