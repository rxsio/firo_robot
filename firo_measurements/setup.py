from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'firo_measurements'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Klaudia Niedziałkowska',
    maintainer_email='klaudiacoding@gmail.com',
    description='Nodes to synchronize robot telemetry from ROS topics with InfluxDB',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_node= firo_measurements.telemetry_node:main'
        ],
    },
)
