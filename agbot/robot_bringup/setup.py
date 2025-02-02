from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name,'world'), glob('world/*')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name,'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name,'config'), glob('config/*')),
        (os.path.join('share', package_name,'params'), glob('params/*')),
        (os.path.join('share', package_name,'maps'), glob('maps/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehrad Mortazavi',
    maintainer_email='smortazavi3@ucmerced.edu',
    description='Robot description and bringup',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_state_pub = robot_bringup.joint_state_pub:main',
            'robot_velocity_pub = robot_bringup.robot_velocity_pub:main',
            'robot_debugger = robot_bringup.debugger:main'
        ],
    },
)
