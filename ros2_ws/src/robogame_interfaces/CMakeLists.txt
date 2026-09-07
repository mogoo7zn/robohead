cmake_minimum_required(VERSION 3.8)
project(robogame_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose2D.msg"
  "msg/BlockDetection.msg"
  "msg/MissionStatus.msg"
)

ament_package()
