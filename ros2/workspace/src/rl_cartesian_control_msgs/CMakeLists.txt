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
