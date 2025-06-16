from setuptools import find_packages, setup
import glob
import os

package_name = 'apriltag_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools',
        'opencv-python',
        'numpy',
        'pupil-apriltags'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='ekwang@mit.edu',
    description='AprilTag detector node for zed2i',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_detector_node = apriltag_detector.apriltag_detector_node:main'
        ],
    },
)
