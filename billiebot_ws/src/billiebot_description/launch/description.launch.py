import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('billiebot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'billiebot.urdf.xacro')

    use_sim = LaunchConfiguration('use_sim')
    use_lidar = LaunchConfiguration('use_lidar')
    use_oakd = LaunchConfiguration('use_oakd')
    use_noir = LaunchConfiguration('use_noir')
    use_thermal = LaunchConfiguration('use_thermal')
    use_mic = LaunchConfiguration('use_mic')
    use_imu = LaunchConfiguration('use_imu')

    robot_description = Command([
        'xacro ', xacro_file,
        ' use_sim:=', use_sim,
        ' use_lidar:=', use_lidar,
        ' use_oakd:=', use_oakd,
        ' use_noir:=', use_noir,
        ' use_thermal:=', use_thermal,
        ' use_mic:=', use_mic,
        ' use_imu:=', use_imu,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_oakd', default_value='true'),
        DeclareLaunchArgument('use_noir', default_value='true'),
        DeclareLaunchArgument('use_thermal', default_value='true'),
        DeclareLaunchArgument('use_mic', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
    ])
