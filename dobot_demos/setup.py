from setuptools import setup

package_name = 'dobot_demos'

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
            'test_gripper = dobot_demos.gripper_client:main',
            'test_homing = dobot_demos.homing_client:main',
            'test_point_to_point = dobot_demos.PTP_client:main',
            'test_suction_cup = dobot_demos.suction_cup_client:main',
            'test_pick_and_place = dobot_demos.pick_and_place:main',
        ],
    },
)

