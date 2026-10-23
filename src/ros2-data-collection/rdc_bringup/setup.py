import os
from glob import glob

from setuptools import setup

package_name = 'rdc_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Launch files and device bringup configuration for the data collection system',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
)
