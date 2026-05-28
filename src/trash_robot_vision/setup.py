from glob import glob
from setuptools import find_packages, setup

package_name = 'trash_robot_vision'

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
    description='VLM trash classification, depth localization, and MJPEG overlay for Trash Robot V3.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vlm_trash_classifier = trash_robot_vision.vlm_trash_classifier:main',
            'pixel_depth_locator = trash_robot_vision.pixel_depth_locator:main',
            'light_mjpeg_streamer = trash_robot_vision.light_mjpeg_streamer:main',
            'yolo_trash_candidate = trash_robot_vision.yolo_trash_candidate:main',
        ],
    },
)
