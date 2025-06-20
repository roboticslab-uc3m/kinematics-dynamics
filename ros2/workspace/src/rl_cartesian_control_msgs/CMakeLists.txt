# https://stackoverflow.com/a/25217937
get_directory_property(_has_parent PARENT_DIRECTORY)

if(_has_parent)
  option(ENABLE_ros2_ws_rl_cartesian_control_msgs "Enable/disable rl_cartesian_control_msgs ROS 2 package" ON)

  if(NOT ENABLE_ros2_ws_rl_cartesian_control_msgs)
    return()
  endif()
endif()

cmake_minimum_required(VERSION 3.19)
project(rl_cartesian_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/Inv.srv
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()

# Temporary workaround for https://github.com/ros2/rosidl/pull/605
if(NOT TARGET rl_cartesian_control_msgs::rl_cartesian_control_msgs__rosidl_typesupport_cpp)
  add_library(rl_cartesian_control_msgs::rl_cartesian_control_msgs__rosidl_typesupport_cpp ALIAS rl_cartesian_control_msgs__rosidl_typesupport_cpp)
endif()
