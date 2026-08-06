from setuptools import setup

package_name = 'estop_keyboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssupath',
    maintainer_email='you@example.com',
    description='Keyboard-driven E-STOP publisher for ForzaETH race stack',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_estop = estop_keyboard.keyboard_estop:main'
        ],
    },
)
