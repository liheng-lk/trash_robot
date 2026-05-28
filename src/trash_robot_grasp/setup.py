from glob import glob
from setuptools import find_packages, setup

package_name = 'trash_robot_grasp'

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
    description='Hand-eye transform and RoArm sorting grasp nodes.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'handeye_target_transformer = trash_robot_grasp.handeye_target_transformer:main',
            'handeye_web_calibrator = trash_robot_grasp.handeye_web_calibrator:main',
            'roarm_sort_grasper = trash_robot_grasp.roarm_sort_grasper:main',
        ],
    },
)
