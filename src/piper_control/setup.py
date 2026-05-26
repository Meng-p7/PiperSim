from setuptools import setup
from glob import glob

package_name = 'piper_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Piper real robot control for ROS2 + MoveIt (ros2_control hardware interface)',
    license='MIT',
    entry_points={
        'console_scripts': [
            # hardware_bridge is now a C++ ros2_control plugin (piper_hardware.so).
            # The Python hardware_bridge.py is kept as a standalone fallback only.
        ],
    },
)
