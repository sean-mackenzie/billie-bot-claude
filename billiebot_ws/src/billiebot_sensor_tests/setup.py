import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'billiebot_sensor_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'foxglove'), glob('foxglove/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='billie',
    maintainer_email='billie@todo.todo',
    description='Bench-test suite for BillieBot sensors, independent of chassis/nav/mission bringup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_sensor_test = billiebot_sensor_tests.orchestrate.run_sensor_test:main',
            'topic_rate_monitor = billiebot_sensor_tests.common.rate_monitor:main',
            'ground_truth_marker_node = billiebot_sensor_tests.common.ground_truth_marker:main',
            'oakd_bench_publisher = billiebot_sensor_tests.oakd.oakd_bench_publisher:main',
            'oakd_preview_overlay = billiebot_sensor_tests.oakd.oakd_preview_overlay:main',
            'analyze_oakd_depth = billiebot_sensor_tests.oakd.analyze_oakd_depth:main',
            'score_oakd_detector = billiebot_sensor_tests.oakd.score_oakd_detector:main',
            'thermal_colorizer = billiebot_sensor_tests.thermal.thermal_colorizer:main',
            'analyze_thermal_frame = billiebot_sensor_tests.thermal.analyze_thermal_frame:main',
            'score_thermal_blob = billiebot_sensor_tests.thermal.score_thermal_blob:main',
            'image_quality_monitor = billiebot_sensor_tests.noir.image_quality_monitor:main',
            'analyze_noir_image = billiebot_sensor_tests.noir.analyze_noir_image:main',
            'audio_capture_node = billiebot_sensor_tests.audio.audio_capture_node:main',
            'analyze_audio = billiebot_sensor_tests.audio.analyze_audio:main',
            'score_audio_classifier = billiebot_sensor_tests.audio.score_audio_classifier:main',
        ],
    },
)
