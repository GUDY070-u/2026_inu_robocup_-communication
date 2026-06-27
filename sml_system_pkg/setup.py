import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sml_system_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
    	('share/ament_index/resource_index/packages',
        	['resource/' + package_name]),
    	('share/' + package_name, ['package.xml']),
    	(os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.com',
    description='SML 시스템 노드 패키지',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                'order_server = sml_system_pkg.order_server:main',
                'sml_planning_node = sml_system_pkg.sml_planning_node:main',
                'sml_manager_node = sml_system_pkg.sml_manager_node:main',
                'mock_nav_node = sml_system_pkg.mock_nav_node:main',
                'mock_arm_node = sml_system_pkg.mock_arm_node:main',
                'mock_wb_node = sml_system_pkg.mock_wb_node:main',
                'eai_task_adapter_node = sml_system_pkg.eai_task_adapter_node:main',
        ],
    },
)

