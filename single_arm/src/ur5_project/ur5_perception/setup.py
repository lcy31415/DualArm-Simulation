from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur5_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcy',
    maintainer_email='lcy31415@gmail.com',
    description='UR5 perception system including camera detection and block classification',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_detector = ur5_perception.color_detector:main',
            'color_detector_handshake = ur5_perception.color_detector_handshake:main',
            'image_viewer = ur5_perception.image_viewer:main',
            'block_classifier_action = ur5_perception.block_classifier_action:main',
        ],
    },
)
