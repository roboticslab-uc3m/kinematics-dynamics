yarp_prepare_plugin(CartesianControlServer
                    CATEGORY device
                    TYPE CartesianControlServer
                    INCLUDE CartesianControlServer.hpp
                    EXTRA_CONFIG WRAPPER=CartesianControlServer
                    DEFAULT ON
                    DEPENDS ENABLE_KinematicRepresentationLib
                    GENERATE_PARSER)

if(NOT SKIP_CartesianControlServer)

    yarp_add_plugin(CartesianControlServer)

    target_sources(CartesianControlServer PRIVATE CartesianControlServer.hpp
                                                  DeviceDriverImpl.cpp
                                                  IWrapperImpl.cpp
                                                  PeriodicThreadImpl.cpp
                                                  RpcResponder.cpp
                                                  StreamResponder.cpp
                                                  CartesianControlServer_ParamsParser.h
                                                  CartesianControlServer_ParamsParser.cpp
                                                  LogComponent.hpp
                                                  LogComponent.cpp)

    target_link_libraries(CartesianControlServer YARP::YARP_os
                                                 YARP::YARP_dev
                                                 ROBOTICSLAB::KinematicRepresentationLib
                                                 ROBOTICSLAB::KinematicsDynamicsInterfaces)

    yarp_install(TARGETS CartesianControlServer
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
