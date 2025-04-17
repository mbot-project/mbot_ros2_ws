from setuptools import find_packages, setup

package_name = 'mbot_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shaw',
    maintainer_email='shaw@todo.todo',
    description='ROS2 bridge for MBot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mbot_ros_bridge_node = mbot_ros_bridge.mbot_ros_bridge_node:main'
        ],
    },
)
