if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_KdlTreeSolver OR ENABLE_KdlTreeSolver))
    message(WARNING "orocos_kdl package not found, disabling KdlTreeSolver")
endif()

yarp_prepare_plugin(KdlTreeSolver
                    CATEGORY device
                    TYPE roboticslab::KdlTreeSolver
                    INCLUDE KdlTreeSolver.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_KdlVectorConverterLib;orocos_kdl_FOUND")

if(NOT SKIP_KdlTreeSolver)

    yarp_add_plugin(KdlTreeSolver KdlTreeSolver.hpp
                                  DeviceDriverImpl.cpp
                                  ICartesianSolverImpl.cpp
                                  LogComponent.hpp
                                  LogComponent.cpp)

    target_link_libraries(KdlTreeSolver YARP::YARP_os
                                        YARP::YARP_dev
                                        ${orocos_kdl_LIBRARIES}
                                        ROBOTICSLAB::KdlVectorConverterLib
                                        ROBOTICSLAB::KinematicsDynamicsInterfaces)

    target_include_directories(KdlTreeSolver PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    yarp_install(TARGETS KdlTreeSolver
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                 YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_KdlTreeSolver OFF CACHE BOOL "Enable/disable KdlTreeSolver device" FORCE)

endif()
