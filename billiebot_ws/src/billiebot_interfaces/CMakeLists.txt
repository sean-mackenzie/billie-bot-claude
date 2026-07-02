cmake_minimum_required(VERSION 3.8)
project(billiebot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/DogDetection3D.msg"
  "msg/DogState.msg"
  "msg/AudioEvent.msg"
  "msg/ThermalBlob.msg"
  "msg/BatteryStatus.msg"
  "msg/MissionStatus.msg"
  # Services
  "srv/EStop.srv"
  "srv/SetMode.srv"
  "srv/GetDogState.srv"
  # Actions
  "action/ApproachDog.action"
  "action/Retreat.action"
  "action/Speak.action"
  "action/DispenseTreat.action"
  "action/PatrolWaypoints.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    builtin_interfaces
)

ament_package()
