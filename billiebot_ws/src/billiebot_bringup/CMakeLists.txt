cmake_minimum_required(VERSION 3.8)
project(billiebot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config scripts udev
  DESTINATION share/${PROJECT_NAME}
)

# Make scripts executable
install(PROGRAMS
  scripts/install_udev_rules.sh
  scripts/check_devices.sh
  scripts/verify_rung_01.sh
  scripts/verify_rung_02.sh
  scripts/verify_rung_03.sh
  scripts/verify_rung_06.sh
  scripts/verify_rung_07.sh
  scripts/verify_rung_12.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
