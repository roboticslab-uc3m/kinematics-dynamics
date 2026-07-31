yarp_prepare_plugin(CartesianControlClientROS2
                    CATEGORY device
                    TYPE CartesianControlClientROS2
                    INCLUDE CartesianControlClientROS2.hpp
                    DEFAULT ON
                    DEPENDS rclcpp_FOUND
                    GENERATE_PARSER)

if(NOT SKIP_CartesianControlClientROS2)

    yarp_add_plugin(CartesianControlClientROS2)

    target_sources(CartesianControlClientROS2 PRIVATE CartesianControlClientROS2.hpp
                                                      DeviceDriverImpl.cpp
                                                      ICartesianControlImpl.cpp
                                                      CartesianControlClientROS2_ParamsParser.h
                                                      CartesianControlClientROS2_ParamsParser.cpp
                                                      LogComponent.hpp
                                                      LogComponent.cpp)

    target_link_libraries(CartesianControlClientROS2 YARP::YARP_os
                                                     YARP::YARP_dev
                                                     rclcpp::rclcpp
                                                     ROBOTICSLAB::KinematicsDynamicsInterfaces
                                                     ROBOTICSLAB::RosUtils)

    yarp_install(TARGETS CartesianControlClientROS2
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
