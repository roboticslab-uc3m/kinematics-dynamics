if(NOT YARP_math_FOUND AND (NOT DEFINED ENABLE_AsibotSolver OR ENABLE_AsibotSolver))
    message(WARNING "YARP_math lib not found, disabling AsibotSolver")
endif()

yarp_prepare_plugin(AsibotSolver
                    CATEGORY device
                    TYPE roboticslab::AsibotSolver
                    INCLUDE AsibotSolver.hpp
                    DEFAULT ON
                    DEPENDS "YARP_math_FOUND;ENABLE_KinematicRepresentationLib")

if(NOT SKIP_AsibotSolver)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(AsibotSolver AsibotSolver.hpp
                                 AsibotSolver.cpp
                                 DeviceDriverImpl.cpp
                                 ICartesianSolverImpl.cpp
                                 AsibotConfiguration.hpp
                                 AsibotConfiguration.cpp
                                 AsibotConfigurationLeastOverallAngularDisplacement.cpp)

    target_link_libraries(AsibotSolver YARP::YARP_OS
                                       YARP::YARP_dev
                                       YARP::YARP_sig
                                       YARP::YARP_math
                                       ROBOTICSLAB::ColorDebug
                                       ROBOTICSLAB::KinematicRepresentationLib
                                       ROBOTICSLAB::KinematicsDynamicsInterfaces)

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS AsibotSolver
                     LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS AsibotSolver
                     LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})

        yarp_install(FILES AsibotSolver.ini
                     DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

else()

    set(ENABLE_AsibotSolver OFF CACHE BOOL "Enable/disable AsibotSolver device" FORCE)

endif()
