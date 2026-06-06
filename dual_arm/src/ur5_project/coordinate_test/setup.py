from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'coordinate_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcy',
    maintainer_email='lcy@todo.todo',
    description='Dual-arm coordinate test GUI',
    license='TODO',
    entry_points={
        'console_scripts': [
            'coordinate_test_gui = coordinate_test.coordinate_test_gui:main',
        ],
    },
)
