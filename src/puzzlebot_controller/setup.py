import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjj',
    maintainer_email='jjjau03@gmail.com',
    description='Puzzlebot differential drive controller and odometry',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'simple_controller = puzzlebot_controller.simple_controller:main',
            'noisy_controller = puzzlebot_controller.noisy_controller:main',
            'twist_relay = puzzlebot_controller.twist_relay:main',
        ],
    },
)
