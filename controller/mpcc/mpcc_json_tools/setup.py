from setuptools import setup
import os

package_name = 'mpcc_json_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='misys',
    maintainer_email='misys@todo.todo',
    description='Generate MPCC track, bounds, and normalization JSON from centerline waypoints',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpcc_json_tools = mpcc_json_tools.mpcc_json_tools:main',
        ],
    },
)
