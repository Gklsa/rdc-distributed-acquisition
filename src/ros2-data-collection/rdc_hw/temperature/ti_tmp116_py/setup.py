import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ti_tmp116_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TI TMP116 temperature sensor driver and ROS2 node (I2C) — complete Python reference implementation',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'tmp116_node = ti_tmp116_py.tmp116_node:main',
        ],
    },
)
