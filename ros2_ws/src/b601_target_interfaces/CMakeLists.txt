cmake_minimum_required(VERSION 3.8)
project(b601_target_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetFrame.msg"
  "msg/JointFeedforward.msg"
  "msg/MitTarget.msg"
  "msg/MitTargetFrame.msg"
  "msg/AuthorizedTarget.msg"
  "msg/CompensationSources.msg"
  "msg/GravityCompensation.msg"
  "msg/LoadEstimate.msg"
  "msg/LoadEstimates.msg"
  "srv/SelectSource.srv"
  "srv/SetAuthority.srv"
  "srv/ReleaseSource.srv"
  "srv/RevokeSource.srv"
  "srv/RenewSource.srv"
  "srv/SetTarget.srv"
  "srv/SetMitTarget.srv"
  "srv/SetGravitySource.srv"
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
