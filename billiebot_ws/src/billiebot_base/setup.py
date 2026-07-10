import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'billiebot_base'

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
    description='Base driver bridge for BillieBot differential drive',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_bridge = billiebot_base.base_bridge:main',
            'mock_scan = billiebot_base.mock_scan:main',
        ],
    },
)
