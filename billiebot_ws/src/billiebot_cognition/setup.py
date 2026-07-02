import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'billiebot_cognition'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'templates'), glob('templates/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='billie',
    maintainer_email='billie@todo.todo',
    description='Cognition nodes for BillieBot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'state_fusion = billiebot_cognition.state_fusion:main',
            'dog_logger = billiebot_cognition.dog_logger:main',
            'daily_report = billiebot_cognition.daily_report:main',
            'report_server = billiebot_cognition.report_server:main',
        ],
    },
)
