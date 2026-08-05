from setuptools import setup
import os
from glob import glob

package_name = 'on_track_sys_id'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
        ('share/' + package_name + '/models', glob('models/Orinnano/*.txt'))
    ],
    install_requires=['setuptools'],
    maintainer='Onur Dikici',
    maintainer_email='odikici@ethz.ch',
    description='ROS 2 package for system identification',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'on_track_sys_id = on_track_sys_id.on_track_sys_id:main',
        ],
    },
)
