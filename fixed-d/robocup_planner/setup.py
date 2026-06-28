from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robocup_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeoch',
    maintainer_email='yeochoony@gmail.com',
    description='RoboCup material pickup and assembly planner',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = robocup_planner.planner_node:main',
        ],
    },
)
