from setuptools import setup

package_name = 'nanospec'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehrad Mortazavi',
    maintainer_email='smortazavi3@ucmerced.edu',
    description='Ros2 driver for nanolambda NSP32',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'NSP32_service_node = nanospec.NSP32_service_node:main',
            'NSP32_client_node = nanospec.NSP32_client_node:main',
            'NSP32_client_triggered = nanospec.NSP32_client_triggered:main'
        ],
    },
)
