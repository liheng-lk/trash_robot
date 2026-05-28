from glob import glob
from setuptools import find_packages, setup

package_name = 'trash_robot_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Mission supervisor for patrol and trash sorting.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_supervisor = trash_robot_mission.mission_supervisor:main',
        ],
    },
)
