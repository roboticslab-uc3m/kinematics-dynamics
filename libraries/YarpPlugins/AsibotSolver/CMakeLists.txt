yarp_prepare_plugin(AsibotSolver
                    CATEGORY device
                    TYPE roboticslab::AsibotSolver
                    INCLUDE AsibotSolver.hpp
                    DEFAULT ON
                    DEPENDS "YARP_HAS_MATH_LIB;ENABLE_KinematicRepresentationLib")

unset(ENABLE_AsibotSolver)

if(NOT DEFINED ENABLE_AsibotSolver)
    set(ENABLE_AsibotSolver ON CACHE BOOL "")
endif()

if(NOT YARP_HAS_MATH_LIB)
    if(ENABLE_AsibotSolver)
        message(WARNING "YARP::YARP_math package not found, disabling AsibotSolver")
    endif()
    if(NOT DEFINED ENABLE_AsibotSolver-STORED)
        set(ENABLE_AsibotSolver-STORED "${ENABLE_AsibotSolver}" CACHE INTERNAL "Stored value of ENABLE_AsibotSolver prior to forcing OFF")
    endif()
    set(ENABLE_AsibotSolver OFF CACHE BOOL "Enable/disable AsibotSolver" FORCE)
elseif(DEFINED ENABLE_AsibotSolver-STORED AND NOT ENABLE_AsibotSolver)
    set(ENABLE_AsibotSolver "${ENABLE_AsibotSolver-STORED}" CACHE BOOL "" FORCE)
    unset(ENABLE_AsibotSolver-STORED CACHE)
endif()

if(ENABLE_AsibotSolver)

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

endif()
