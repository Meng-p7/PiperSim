from setuptools import setup
from glob import glob

package_name = 'piper_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.srdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Hand-eye calibration for Piper robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_node = piper_calibration.calibration_node:main',
        ],
    },
)
