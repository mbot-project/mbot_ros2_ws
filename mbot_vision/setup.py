from setuptools import find_packages, setup
from glob import glob

package_name = 'mbot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/calibration', glob('calibration/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shaw',
    maintainer_email='xssun@umich.edu',
    description='MBot vision package for camera, rectification, and AprilTag detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_marker = mbot_vision.calibration_marker:main',
        ],
    },
)