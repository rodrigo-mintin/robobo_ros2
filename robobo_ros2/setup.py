from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'robobo_ros2'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        # Required for ROS 2 package discovery
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Package metadata
        ('share/' + package_name, ['package.xml']),

        # Install launch files (if any)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # Install service definitions
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
        
        # Install config files (YAML)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.yml')),
    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='eden',
    maintainer_email='rodrigo.martin@mintforpeople.com',

    description='Robobo ROS2 interface node',

    license='TODO',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'robobo_container = robobo_ros2.robobo_container:main',
            'robobo_container.py = robobo_ros2.robobo_container:main',
            'sample_demo = robobo_ros2.sample_demo:main',
            'sample_demo.py = robobo_ros2.sample_demo:main',
        ],
    },
)