import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'billiebot_audio'

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
    description='Audio classification and speaker nodes for BillieBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_classifier = billiebot_audio.audio_classifier:main',
            'speaker_node = billiebot_audio.speaker_node:main',
        ],
    },
)
