# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores & Raul de Santos Rico
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

if(NOT orocos_kdl_FOUND AND (NOT DEFINED ENABLE_KdlSolver OR ENABLE_KdlSolver))
    message(WARNING "orocos_kdl package not found, disabling KdlSolver")
endif()

yarp_prepare_plugin(KdlSolver
                    CATEGORY device
                    TYPE roboticslab::KdlSolver
                    INCLUDE KdlSolver.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_ScrewTheoryLib;ENABLE_KdlVectorConverterLib;ENABLE_KinematicRepresentationLib;orocos_kdl_FOUND")

if(NOT SKIP_KdlSolver)

    set(CMAKE_INCLUDE_CURRENT_DIR TRUE) # yarp plugin builder needs this

    yarp_add_plugin(KdlSolver KdlSolver.hpp
                              DeviceDriverImpl.cpp
                              ICartesianSolverImpl.cpp
                              ChainFkSolverPos_ST.hpp
                              ChainFkSolverPos_ST.cpp
                              ChainIkSolverPos_ST.hpp
                              ChainIkSolverPos_ST.cpp
                              ChainIkSolverPos_ID.hpp
                              ChainIkSolverPos_ID.cpp)

    target_link_libraries(KdlSolver YARP::YARP_OS
                                    YARP::YARP_dev
                                    YARP::YARP_sig
                                    ${orocos_kdl_LIBRARIES}
                                    ROBOTICSLAB::ColorDebug
                                    ROBOTICSLAB::ScrewTheoryLib
                                    ROBOTICSLAB::KdlVectorConverterLib
                                    ROBOTICSLAB::KinematicRepresentationLib
                                    ROBOTICSLAB::KinematicsDynamicsInterfaces)

    target_include_directories(KdlSolver PRIVATE ${orocos_kdl_INCLUDE_DIRS})

    if(NOT YARP_VERSION_SHORT VERSION_LESS 3.2)
        yarp_install(TARGETS KdlSolver
                     LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR}
                     YARP_INI DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})
    else()
        yarp_install(TARGETS KdlSolver
                     LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                     ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})
    
        yarp_install(FILES KdlSolver.ini
                     DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})
    endif()

else()

    set(ENABLE_KdlSolver OFF CACHE BOOL "Enable/disable KdlSolver device" FORCE)

endif()
