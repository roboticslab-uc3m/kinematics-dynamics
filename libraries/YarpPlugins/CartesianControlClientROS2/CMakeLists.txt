if(NOT DEFINED ENABLE_CartesianControlClientROS2 OR ENABLE_CartesianControlClientROS2)
    if(NOT rclcpp_FOUND)
        message(WARNING "rclcpp package not found, disabling CartesianControlClientROS2")
    elseif(NOT rclcpp_action_FOUND)
        message(WARNING "rclcpp_action package not found, disabling CartesianControlClientROS2")
    elseif(NOT std_srvs_FOUND)
        message(WARNING "std_srvs package not found, disabling CartesianControlClientROS2")
    elseif(NOT geometry_msgs_FOUND)
        message(WARNING "geometry_msgs package not found, disabling CartesianControlClientROS2")
    elseif(NOT (rl_cartesian_control_msgs_FOUND OR TARGET rl_cartesian_control_msgs::rl_cartesian_control_msgs__rosidl_typesupport_cpp))
        message(WARNING "rl_cartesian_control_msgs package not found, disabling CartesianControlClientROS2")
    elseif(NOT orocos_kdl_FOUND)
        message(WARNING "orocos_kdl package not found, disabling CartesianControlClientROS2")
    endif()
endif()

yarp_prepare_plugin(CartesianControlClientROS2
                    CATEGORY device
                    TYPE CartesianControlClientROS2
                    INCLUDE CartesianControlClientROS2.hpp
                    DEFAULT ON
                    DEPENDS "rclcpp_FOUND;rclcpp_action_FOUND;std_srvs_FOUND;geometry_msgs_FOUND;rl_cartesian_control_msgs_FOUND OR TARGET rl_cartesian_control_msgs::rl_cartesian_control_msgs__rosidl_typesupport_cpp;orocos_kdl_FOUND"
                    GENERATE_PARSER)

if(NOT SKIP_CartesianControlClientROS2)

    yarp_add_plugin(CartesianControlClientROS2)

    target_sources(CartesianControlClientROS2 PRIVATE CartesianControlClientROS2.hpp
                                                      CartesianControlClientROS2.cpp
                                                      DeviceDriverImpl.cpp
                                                      ICartesianControlImpl.cpp
                                                      CartesianControlClientROS2_ParamsParser.h
                                                      CartesianControlClientROS2_ParamsParser.cpp
                                                      LogComponent.hpp
                                                      LogComponent.cpp)

    target_link_libraries(CartesianControlClientROS2 YARP::YARP_os
                                                     YARP::YARP_dev
                                                     rclcpp::rclcpp
                                                     rclcpp_action::rclcpp_action
                                                     std_srvs::std_srvs__rosidl_typesupport_cpp
                                                     geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
                                                     rl_cartesian_control_msgs::rl_cartesian_control_msgs__rosidl_typesupport_cpp
                                                     ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                     ROBOTICSLAB::Ros2Utils
                                                     ${orocos_kdl_LIBRARIES})

    target_include_directories(CartesianControlClientROS2 PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    yarp_install(TARGETS CartesianControlClientROS2
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_CartesianControlClientROS2 OFF CACHE BOOL "Enable/disable CartesianControlClientROS2 device" FORCE)

endif()
