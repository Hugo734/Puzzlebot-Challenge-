import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament index marker
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        ),
        # config files
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jordan',
    maintainer_email='jordanpalafoxs@gmail.com',
    description='Vision-based perception for the Puzzlebot AMR',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'alignment_node = perception.alignment_node:main',
            'camera_calibration = perception.camera_calibration:main',
            'qr_pose_viewer = perception.qr_pose_viewer:main',
            'qr_alignment = perception.qr_alignment:main',
            'qr_path_alignment = perception.qr_path_alignment:main',
            'qr_quad_alignment = perception.qr_quad_alignment:main',
        ],
    },
)
