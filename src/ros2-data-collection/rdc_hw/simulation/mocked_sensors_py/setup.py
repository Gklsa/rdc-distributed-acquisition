from setuptools import find_packages, setup

package_name = 'mocked_sensors_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Python mock sensor publishers for simulation and standalone testing',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'mock_imu_publisher = mocked_sensors_py.mock_imu_publisher:main',
            'mock_temperature_publisher = mocked_sensors_py.mock_temperature_publisher:main',
            'mock_camera_publisher = mocked_sensors_py.mock_camera_publisher:main',
        ],
    },
)
