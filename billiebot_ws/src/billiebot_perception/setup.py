import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'billiebot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='billie',
    maintainer_email='billie@todo.todo',
    description='Perception nodes for BillieBot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oakd_dog_detector = billiebot_perception.oakd_dog_detector:main',
            'dog_locator = billiebot_perception.dog_locator:main',
            'thermal_node = billiebot_perception.thermal_node:main',
            'noir_cam_node = billiebot_perception.noir_cam_node:main',
        ],
    },
)
