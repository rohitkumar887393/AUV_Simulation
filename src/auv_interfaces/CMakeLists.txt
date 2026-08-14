cmake_minimum_required(VERSION 3.8)
project(auv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Orientation.msg"
  "msg/Depth.msg"
  "msg/Leak.msg"
  "msg/Battery.msg"
  "msg/DepthHold.msg"
  "msg/Odometry.msg"
  "msg/Mission.msg"
  "msg/ActuatorCmds.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
