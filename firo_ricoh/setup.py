from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'firo_ricoh'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Brzeziński',
    maintainer_email='gabriel@gabrielb.dev',
    description='Ricoh Theta camera image capture',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'image_capture = firo_ricoh.image_capture:main'
        ],
    },
)
