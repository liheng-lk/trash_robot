from setuptools import find_packages, setup

package_name = 'trash_robot_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['static/*'], package_name + '.static': ['index.html']},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/web_console.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='Simple bilingual WebUI demo for Trash Robot V3.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'web_console = trash_robot_web.web_console:main',
        ],
    },
)
