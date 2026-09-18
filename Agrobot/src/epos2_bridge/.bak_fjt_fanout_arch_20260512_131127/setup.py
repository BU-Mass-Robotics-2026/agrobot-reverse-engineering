import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'epos2_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'config', 'joints'), glob('config/joints/*.yaml')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [p for p in glob('launch/*.py') if os.path.isfile(p)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics-club',
    maintainer_email='robotics-club@example.com',
    description='EPOS2 J3 bridge for IPM + MoveIt integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'epos2_joint_bridge = epos2_bridge.epos2_joint_bridge:main',
            'epos2_j3_bridge = epos2_bridge.epos2_j3_bridge:main',
            'epos2_arm_controller = epos2_bridge.epos2_arm_controller:main',
        ],
    },
)