yarp_prepare_plugin(CartesianControlClient
                    CATEGORY device
                    TYPE CartesianControlClient
                    INCLUDE CartesianControlClient.hpp
                    DEFAULT ON
                    GENERATE_PARSER)

if(NOT SKIP_CartesianControlClient)

    yarp_add_plugin(CartesianControlClient)

    target_sources(CartesianControlClient PRIVATE CartesianControlClient.hpp
                                                  DeviceDriverImpl.cpp
                                                  ICartesianControlImpl.cpp
                                                  FkStreamResponder.cpp
                                                  CartesianControlClient_ParamsParser.h
                                                  CartesianControlClient_ParamsParser.cpp
                                                  LogComponent.hpp
                                                  LogComponent.cpp)

    target_link_libraries(CartesianControlClient YARP::YARP_os
                                                 YARP::YARP_dev
                                                 ROBOTICSLAB::KinematicsDynamicsInterfaces)

    yarp_install(TARGETS CartesianControlClient
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
