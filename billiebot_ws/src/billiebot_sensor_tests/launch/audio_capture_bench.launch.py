"""UT-AUD-01: USB/ALSA enumeration, WAV capture, and signal-quality bench. Run 3x
separately with capture_label:=ambient|speech|impulse.

Starts the bench-only audio_capture_node (continuous InputStream + bounded queue -- never
the production audio_classifier's capture path), optional rosbag2 recording of its
diagnostics topic, and foxglove_bridge.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from billiebot_sensor_tests.common.launch_helpers import (
    declare_common_bench_args,
    duration_shutdown_action,
    finalize_on_shutdown_action,
    foxglove_bridge_action,
    manifest_bootstrap_action,
    record_bag_action,
)

_TOPICS = {'diagnostics': '/bench/audio/diagnostics'}


def generate_launch_description():
    actions = declare_common_bench_args(default_duration_sec='4')
    actions += [
        DeclareLaunchArgument('mock', default_value='false'),
        DeclareLaunchArgument('capture_label', default_value='capture'),
        DeclareLaunchArgument('capture_duration_sec', default_value='3.0'),
        DeclareLaunchArgument('sample_rate', default_value='16000'),
        DeclareLaunchArgument('channels', default_value='2'),
        DeclareLaunchArgument('audio_device_name', default_value='XVF3800'),
        DeclareLaunchArgument('audio_device_serial', default_value=''),
    ]

    actions.append(manifest_bootstrap_action('UT-AUD-01', 'audio', 'reSpeaker XVF3800'))

    actions.append(Node(
        package='billiebot_sensor_tests',
        executable='audio_capture_node',
        name='audio_capture_node',
        parameters=[{
            'mock': LaunchConfiguration('mock'),
            'sample_rate': LaunchConfiguration('sample_rate'),
            'channels': LaunchConfiguration('channels'),
            'capture_duration_sec': LaunchConfiguration('capture_duration_sec'),
            'capture_label': LaunchConfiguration('capture_label'),
            'output_dir': PathJoinSubstitution([LaunchConfiguration('results_dir'), 'exports']),
            'device_name_substring': LaunchConfiguration('audio_device_name'),
            'fail_on_missing_device': LaunchConfiguration('fail_on_missing_device'),
        }],
        output='screen',
    ))

    actions += record_bag_action(list(_TOPICS.values()))
    actions.append(foxglove_bridge_action())
    actions += duration_shutdown_action()
    actions.append(finalize_on_shutdown_action())

    return LaunchDescription(actions)
