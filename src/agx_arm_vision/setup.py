from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agx_arm_vision'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='s1',
    maintainer_email='s1@todo.todo',
    description='RealSense ArUco vision and grasp execution for the AGX Nero arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_grasp_node = agx_arm_vision.vision_grasp_node:main',
            'virtual_aruco_pub = agx_arm_vision.virtual_aruco_pub:main',
            'virtual_depth_camera = agx_arm_vision.virtual_depth_camera:main',
            'grasp_executor = agx_arm_vision.grasp_executor:main',
            'manual_arm_move = agx_arm_vision.manual_arm_move:main',
            'manual_gripper = agx_arm_vision.manual_gripper:main',
        ],
    },
)
