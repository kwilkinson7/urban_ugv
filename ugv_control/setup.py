from setuptools import setup
import os
from glob import glob

package_name = 'ugv_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kendall',
    maintainer_email='kendall@digitalmindrobotics.com',
    description='ROS2 UGV control package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motion_node = ugv_control.motion_node:main'
        ],
    },
)
