from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wearable_robot_upper_limb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_control = wearable_robot_upper_limb.robot_control:main',
            'robot_command = wearable_robot_upper_limb.robot_command:main',
            'load_cell = wearable_robot_upper_limb.load_cell:main',
            'dynamixel_pub = wearable_robot_upper_limb.dynamixel_pub:main',
            'robot_state = wearable_robot_upper_limb.robot_state:main',
        ],
    },
)
