from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anduril_manual_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent',
    maintainer_email='vcadicamo@icloud.com',
    description='Manual keyboard teleoperation publishing AttitudeTarget setpoints.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manual_control = anduril_manual_control.manual_control_node:main',
        ],
    },
)