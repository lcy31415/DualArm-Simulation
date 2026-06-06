from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ur5_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcy',
    maintainer_email='lcy31415@gmail.com',
    description='YOLOv8 OBB overhead-camera detection for UR5 workspace',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detect_node = ur5_perception.detect_node:main',
        ],
    },
)
