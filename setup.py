from setuptools import setup
import os
from glob import glob

package_name = 'robile_project_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yesmin',
    maintainer_email='yesmin@todo.todo',
    description='Robile Project Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localization = robile_project_planner.localization_node:main',
            'final_navigation = robile_project_planner.final_navigation:main',
            'exploration = robile_project_planner.exploration_node:main',
        ],
    },
)
