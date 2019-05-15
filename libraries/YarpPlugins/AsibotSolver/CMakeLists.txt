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

    include_directories(${CMAKE_CURRENT_SOURCE_DIR}) # yarp plugin builder needs this

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
                                       KinematicRepresentationLib
                                       KinematicsDynamicsInterfaces)

    yarp_install(TARGETS AsibotSolver
                 COMPONENT runtime
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES AsibotSolver.ini
                 COMPONENT runtime
                 DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

else()

    set(ENABLE_AsibotSolver OFF CACHE BOOL "Enable/disable AsibotSolver device" FORCE)

endif()
