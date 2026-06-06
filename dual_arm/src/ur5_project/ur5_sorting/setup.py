from setuptools import find_packages, setup

package_name = 'ur5_sorting'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lcy',
    maintainer_email='lcy@todo.todo',
    description='双臂分拣任务管理器',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sorting_demo_1 = ur5_sorting.sorting_demo_1:main',
        ],
    },
)
