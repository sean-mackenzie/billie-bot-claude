# my_bot

ROS 2 package for a small differential-drive robot that can run in Gazebo or on
real hardware. The package contains the robot description, simulation worlds,
Gazebo bridge configuration, RViz layouts, launch files for common workflows,
and a Python serial base driver for an Arduino-style motor controller.

## Repository layout

- `description/` - Xacro robot description, core chassis geometry, inertial
  macros, lidar/camera attachments, and Gazebo control plugins.
- `launch/` - ROS 2 launch files for simulation, robot state publishing, teleop,
  lidar, camera, SLAM, localization, and real-robot bringup.
- `config/` - Gazebo bridge configs plus base driver, SLAM Toolbox, AMCL, and
  Nav2 parameter files.
- `rviz/` - RViz layouts for odometry, lidar, camera/lidar, and Nav2 views.
- `scripts/` - Installable Python nodes:
  - `diff_drive_base.py` subscribes to `/cmd_vel`, talks to the motor controller
    over serial, publishes `/joint_states` and `/odom`, and broadcasts
    `odom -> base_link`.
  - `wait_for_sim_ready.py` waits for simulation topics/services before
    downstream startup.
- `worlds/` - Gazebo SDF worlds, including an empty world and living-room test
  worlds.

## Main capabilities

- Gazebo simulation through `ros_gz_sim`, `ros_gz_bridge`, and `ros_gz_image`.
- Optional simulated lidar and camera selected through launch arguments.
- Real RPLidar A1 bringup through `rplidar_ros`.
- Raspberry Pi camera launch through `camera_ros`.
- Keyboard teleop through `teleop_twist_keyboard`.
- SLAM through `slam_toolbox`.
- Localization and Nav2 parameter files for map-based navigation.
- Serial differential-drive base driver configured by `config/base_driver.yaml`.

## Dependencies

This package is built with `ament_cmake` and targets ROS 2. The current launch
files and comments reference ROS 2 Jazzy-era Nav2 paths.

Package dependencies declared in `package.xml`:

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `tf2_ros`
- `robot_state_publisher`
- `xacro`
- `teleop_twist_keyboard`
- `python3-serial`
- `rplidar_ros`
- `camera_ros`
- `image_transport`

Additional packages used by launch/config files but not declared in
`package.xml` include:

- `ros_gz_sim`
- `ros_gz_bridge`
- `ros_gz_image`
- `rviz2`
- `slam_toolbox`
- Nav2 packages such as `nav2_map_server`, `nav2_amcl`, `nav2_controller`,
  `nav2_planner`, `nav2_behaviors`, `nav2_bt_navigator`,
  `nav2_waypoint_follower`, and `nav2_lifecycle_manager`

## Build

From the root of the ROS 2 workspace:

```bash
colcon build --packages-select my_bot
source install/setup.bash
```

If you are already inside this package directory, build from the workspace root
that contains `src/my_bot`.

## Simulation

Launch the base simulation with no optional sensors:

```bash
ros2 launch my_bot launch_sim.launch.py
```

Launch specific simulation presets:

```bash
ros2 launch my_bot sim_lidar.launch.py
ros2 launch my_bot sim_camera.launch.py
ros2 launch my_bot sim_lidar_camera.launch.py
ros2 launch my_bot sim_teleop.launch.py
ros2 launch my_bot sim_full.launch.py
```

`launch_sim.launch.py` accepts these arguments:

- `world` - SDF world file to load. Defaults to `worlds/living_room.sdf`.
- `use_lidar` - Include simulated lidar and bridge `/scan`.
- `use_camera` - Include simulated camera and bridge `/camera/image_raw`.
- `use_teleop` - Start keyboard teleop.

Example with a different world:

```bash
ros2 launch my_bot launch_sim.launch.py \
  world:=$(ros2 pkg prefix my_bot)/share/my_bot/worlds/empty.world \
  use_lidar:=true \
  use_camera:=true
```

## Real robot

Base driver only:

```bash
ros2 launch my_bot real_drive_base.launch.py
```

Base driver plus RPLidar:

```bash
ros2 launch my_bot real_drive_lidar.launch.py
```

Base driver plus RPLidar and camera:

```bash
ros2 launch my_bot real_drive_full.launch.py
```

The base driver reads its defaults from `config/base_driver.yaml`. Important
parameters include:

- `port` - serial device for the motor controller.
- `baudrate` - serial baud rate.
- `wheel_radius`
- `wheel_separation`
- `encoder_ticks_per_rev`
- motor and encoder sign parameters.

Several RPLidar launch files default to:

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

`config/base_driver.yaml` currently defaults the motor controller to:

```text
/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
```

Update those paths for the connected hardware before launching on a different
machine.

## SLAM, localization, and navigation

Simulation SLAM:

```bash
ros2 launch my_bot sim_slam.launch.py
```

Real robot SLAM:

```bash
ros2 launch my_bot real_drive_slam.launch.py
```

Localization and Nav2 files are present, but some launch files still contain
machine-local assumptions:

- `sim_localization.launch.py` and `sim_nav.launch.py` hard-code the map path
  `/home/sean-mackenzie/my_world_map.yaml`.
- `sim_nav.launch.py` and `sim_nav_only.launch.py` reference
  `config/nav2_params10.yaml`, while this repository currently contains
  `config/nav2_params.yaml`.
- `real_drive_nav.launch.py` launches map-based Nav2 bringup and requires the
  Nav2 packages listed in `package.xml` to be installed in the active ROS 2
  environment.

Adjust those paths or launch files before relying on the Nav2 bringup flow.

## Useful topics and frames

- `/cmd_vel` - velocity command input for teleop/Nav2.
- `/joint_states` - wheel joint state output from the base driver or simulator.
- `/odom` - odometry output.
- `/scan` - lidar output when lidar is enabled.
- `/camera/image_raw` - camera image output when camera is enabled.
- `odom`, `base_link`, `laser_frame`, and camera frames are provided through the
  robot description and launch configuration.

## Notes

- `teleop.launch.py` starts `teleop_twist_keyboard` with `xterm -e`, so `xterm`
  must be available for that launcher.
- The real base driver expects firmware that accepts `m <left> <right>`,
  `e`, and `r` serial commands and replies with encoder counts or `OK`.
- The launch files are actively tailored to the author's robot and machine.
  Treat serial device IDs, map paths, and Nav2 parameter filenames as
  site-specific configuration.
