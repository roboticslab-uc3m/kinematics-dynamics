if(NOT DEFINED ENABLE_CartesianControlServerROS2 OR ENABLE_CartesianControlServerROS2)
    if(NOT orocos_kdl_FOUND)
        message(WARNING "orocos_kdl package not found, disabling CartesianControlServerROS2")
    elseif(NOT rclcpp_FOUND)
        message(WARNING "rclcpp package not found, disabling CartesianControlServerROS2")
    elseif(NOT std_msgs_FOUND)
        message(WARNING "std_msgs package not found, disabling CartesianControlServerROS2")
    elseif(NOT std_srvs_FOUND)
        message(WARNING "std_srvs package not found, disabling CartesianControlServerROS2")
    elseif(NOT geometry_msgs_FOUND)
        message(WARNING "geometry_msgs package not found, disabling CartesianControlServerROS2")
    elseif(NOT (rl_cartesian_control_msgs_FOUND OR TARGET rl_cartesian_control_msgs::rl_cartesian_control_msgs__rosidl_typesupport_cpp))
        message(WARNING "rl_cartesian_control_msgs package/target not found, disabling CartesianControlServerROS2")
    endif()
endif()

yarp_prepare_plugin(CartesianControlServerROS2
                    CATEGORY device
                    TYPE CartesianControlServerROS2
                    INCLUDE CartesianControlServerROS2.hpp
                    EXTRA_CONFIG WRAPPER=CartesianControlServerROS2
                    DEFAULT ON
                    DEPENDS "orocos_kdl_FOUND;rclcpp_FOUND;std_msgs_FOUND;std_srvs_FOUND;geometry_msgs_FOUND;rl_cartesian_control_msgs_FOUND OR TARGET rl_cartesian_control_msgs::rl_cartesian_control_msgs__rosidl_typesupport_cpp"
                    GENERATE_PARSER)

if(NOT SKIP_CartesianControlServerROS2)

    yarp_add_plugin(CartesianControlServerROS2)

    target_sources(CartesianControlServerROS2 PRIVATE CartesianControlServerROS2.hpp
                                                      CartesianControlServerROS2.cpp
                                                      PeriodicThreadImpl.cpp
                                                      DeviceDriverImpl.cpp
                                                      IWrapperImpl.cpp
                                                      Spinner.hpp
                                                      Spinner.cpp
                                                      CartesianControlServerROS2_ParamsParser.h
                                                      CartesianControlServerROS2_ParamsParser.cpp
                                                      LogComponent.hpp
                                                      LogComponent.cpp)

    target_link_libraries(CartesianControlServerROS2 YARP::YARP_os
                                                     YARP::YARP_dev
                                                     rclcpp::rclcpp
                                                     std_msgs::std_msgs__rosidl_typesupport_cpp
                                                     std_srvs::std_srvs__rosidl_typesupport_cpp
                                                     geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
                                                     rl_cartesian_control_msgs::rl_cartesian_control_msgs__rosidl_typesupport_cpp
                                                     ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                     ${orocos_kdl_LIBRARIES})

    target_include_directories(CartesianControlServerROS2 PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    yarp_install(TARGETS CartesianControlServerROS2
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_CartesianControlServerROS2 OFF CACHE BOOL "Enable/disable CartesianControlServerROS2 device" FORCE)

endif()
