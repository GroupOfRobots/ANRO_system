from setuptools import setup

package_name = 'dobot_end_effector'

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
    maintainer='jan',
    maintainer_email='jan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_server = dobot_end_effector.gripper_server:main',
            'suction_cup_server = dobot_end_effector.suction_cup_server:main',
        ],
    },
)
