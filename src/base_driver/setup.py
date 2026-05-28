from setuptools import setup

package_name = 'base_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/base.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@example.com',
    description='Serial base driver for garbage robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_base_node = base_driver.serial_base_node:main',
        ],
    },
)
