from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament to find the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jjj',
    maintainer_email='jjjau03@gmail.com',
    description='Monte Carlo Localization for the Puzzlebot',
    license='TODO',
    entry_points={
        'console_scripts': [
            'kinematic_simulator = puzzlebot_localization.kinematic_simulator:main',
            'dead_reckoning     = puzzlebot_localization.dead_reckoning:main',
            'tf_broadcaster     = puzzlebot_localization.tf_broadcaster:main',
            'mcl_node = puzzlebot_localization.mcl_node:main',
        ],
    },
)
