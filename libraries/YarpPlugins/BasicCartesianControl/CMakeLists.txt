yarp_prepare_plugin(BasicCartesianControl
                    CATEGORY device
                    TYPE roboticslab::BasicCartesianControl
                    INCLUDE BasicCartesianControl.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_KdlVectorConverterLib;orocos_kdl_FOUND"
                    EXTRA_CONFIG WRAPPER=CartesianControlServer)

if(NOT SKIP_BasicCartesianControl)

    yarp_add_plugin(BasicCartesianControl BasicCartesianControl.hpp
                                          BasicCartesianControl.cpp
                                          DeviceDriverImpl.cpp
                                          ICartesianControlImpl.cpp
                                          PeriodicThreadImpl.cpp
                                          LogComponent.hpp
                                          LogComponent.cpp)

    target_link_libraries(BasicCartesianControl YARP::YARP_os
                                                YARP::YARP_dev
                                                ${orocos_kdl_LIBRARIES}
                                                ROBOTICSLAB::KdlVectorConverterLib
                                                ROBOTICSLAB::KinematicsDynamicsInterfaces)

    target_include_directories(BasicCartesianControl PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    yarp_install(TARGETS BasicCartesianControl
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
