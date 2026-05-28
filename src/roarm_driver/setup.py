from setuptools import find_packages, setup

package_name = 'roarm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dudu',
    maintainer_email='dudu@todo.todo',
    description='ROS 2 RoArm hardware driver for Trash Robot V3.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'roarm_driver = roarm_driver.roarm_driver:main',
        ],
    },
)
