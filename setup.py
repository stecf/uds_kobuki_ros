import os
from glob import glob
from setuptools import setup

package_name = 'uds_kobuki_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'deploy/run.bat', 'deploy/Kobuki.ico', 'deploy/run.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fstec',
    maintainer_email='filip.stec@stuba.sk',
    description='ROS 2 runtime software for Kobuki, Yujin Robot\'s mobile research base.',
    license='gplv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uds_kobuki_ros = uds_kobuki_ros.uds_kobuki_ros:main',
        ],
    },
)
