cmake_minimum_required(VERSION 3.8)
project(b601_target_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetFrame.msg"
  "msg/JointFeedforward.msg"
  "msg/MitTarget.msg"
  "msg/MitTargetFrame.msg"
  "msg/AuthorizedTarget.msg"
  "msg/CompensationSources.msg"
  "srv/SelectSource.srv"
  "srv/SetAuthority.srv"
  "srv/ReleaseSource.srv"
  "srv/RevokeSource.srv"
  "srv/RenewSource.srv"
  "srv/SetTarget.srv"
  "srv/SetMitTarget.srv"
  DEPENDENCIES sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
