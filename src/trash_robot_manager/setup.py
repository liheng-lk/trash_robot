from setuptools import find_packages, setup

package_name = 'trash_robot_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Process lifecycle and system status manager for Trash Robot V3.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_manager = trash_robot_manager.robot_manager:main',
        ],
    },
)
