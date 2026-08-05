import os
from glob import glob

from setuptools import setup

package_name = 'rl_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='misys',
    maintainer_email='thdgkgus2001@gmail.com',
    description='DACER++ diffusion policy controller for the F1TENTH race stack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_controller = rl_controller.rl_controller_node:main',
        ],
    },
    scripts=['scripts/check_rl_setup.py'],
)
