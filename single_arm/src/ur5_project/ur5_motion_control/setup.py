from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5_motion_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcy',
    maintainer_email='lcy31415@gmail.com',
    description='UR5 motion control using MoveIt',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 注册节点名称
            'motion_controller = ur5_motion_control.motion_controller:main',
            'motion_controller_action = ur5_motion_control.motion_controller_action:main',
            'test_action_client = ur5_motion_control.test_action_client:main',
        ],
    },
)
