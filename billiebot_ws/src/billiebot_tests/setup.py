import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'billiebot_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh') + glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='billie',
    maintainer_email='billie@todo.todo',
    description='Integration and acceptance tests for BillieBot',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
