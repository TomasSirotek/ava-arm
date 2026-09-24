from glob import glob

from setuptools import find_packages, setup

package_name = 'ava_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/markers', glob('markers/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omar Draidrya',
    maintainer_email='omar@example.com',
    description='Computer Vision package for ArUco marker detection and '
                'overhead-camera pick-and-place pose estimation for ava.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = ava_vision.camera_node:main',
            'aruco_detector_node = ava_vision.aruco_detector_node:main',
            'camera_calibration_node = ava_vision.camera_calibration_node:main',
            'generate_markers = ava_vision.generate_markers:main',
            'cube_pose_publisher = ava_vision.cube_pose_publisher_node:main',
            'calibrate_extrinsics = ava_vision.calibrate_extrinsics:main',
        ],
    },
)
