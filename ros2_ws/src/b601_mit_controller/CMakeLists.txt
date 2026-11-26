cmake_minimum_required(VERSION 3.16)
project(b601_mit_controller CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(b601_description REQUIRED)
find_package(b601_target_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(b601_mit_controller SHARED src/b601_mit_controller.cpp)
target_include_directories(b601_mit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${b601_description_INCLUDE_DIRS})
target_link_libraries(b601_mit_controller PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  realtime_tools::realtime_tools
  ${b601_target_interfaces_TARGETS}
  ${std_msgs_TARGETS})

install(TARGETS b601_mit_controller
  EXPORT export_b601_mit_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES b601_mit_controller_plugins.xml DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(controller_interface b601_mit_controller_plugins.xml)

ament_export_include_directories(include)
ament_export_dependencies(b601_description b601_target_interfaces)
ament_export_libraries(b601_mit_controller)
ament_export_targets(export_b601_mit_controller)
ament_package()
