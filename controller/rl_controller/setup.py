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
        # 가중치를 install 공간에도 넣는다. rl_controller.yaml 의 checkpoint 가
        # 상대경로면 share/rl_controller/models/ 기준으로 해석되므로, 이게 빠지면
        # 학습 워크스페이스 경로에 다시 의존하게 된다.
        # models/ 는 최신 런의 pow.pt / cvar.pt 만 두는 평평한 디렉터리다 (날짜 폴더 없음)
        # -> 가중치를 갱신할 때 이 규칙을 고칠 일이 없다.
        (os.path.join('share', package_name, 'models'),
            glob(os.path.join('models', '*.pt')) + glob(os.path.join('models', '*.json'))
            + glob(os.path.join('models', '*.md'))),
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
