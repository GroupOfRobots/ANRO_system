from setuptools import setup
import os
from glob import glob

package_name = 'dobot_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'examples'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkaniuka',
    maintainer_email='jkaniuka@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_validator_server = dobot_kinematics.trajectory_validator_server:main',
        ],
    },
)
