from glob import glob

from setuptools import find_packages, setup

package_name = 'ava_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='od',
    maintainer_email='od@todo.todo',
    description='Gamepad teleop nodes for the AVA arm',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gamepad_cartesian_node = ava_control.gamepad_cartesian_node:main',
            'gamepad_calibrate_node = ava_control.gamepad_calibrate_node:main',
            'gripper_mimic_bridge = ava_control.gripper_mimic_bridge:main',
        ],
    },
)
