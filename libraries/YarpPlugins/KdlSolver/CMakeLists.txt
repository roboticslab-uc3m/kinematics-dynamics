# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores & Raul de Santos Rico
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(KdlSolver
                    CATEGORY device
                    TYPE roboticslab::KdlSolver
                    INCLUDE KdlSolver.hpp
                    DEFAULT ON
                    DEPENDS "ENABLE_KdlVectorConverterLib;ENABLE_KinematicRepresentationLib;orocos_kdl_FOUND")

unset(ENABLE_KdlSolver)

if(NOT DEFINED ENABLE_KdlSolver)
    set(ENABLE_KdlSolver ON CACHE BOOL "")
endif()

if(NOT orocos_kdl_FOUND)
    if(ENABLE_KdlSolver)
        message(WARNING "orocos_kdl package not found, disabling KdlSolver")
    endif()
    if(NOT DEFINED ENABLE_KdlSolver-STORED)
        set(ENABLE_KdlSolver-STORED "${ENABLE_KdlSolver}" CACHE INTERNAL "Stored value of ENABLE_KdlSolver prior to forcing OFF")
    endif()
    set(ENABLE_KdlSolver OFF CACHE BOOL "Enable/disable KdlSolver" FORCE)
elseif(DEFINED ENABLE_KdlSolver-STORED AND NOT ENABLE_KdlSolver)
    set(ENABLE_KdlSolver "${ENABLE_KdlSolver-STORED}" CACHE BOOL "" FORCE)
    unset(ENABLE_KdlSolver-STORED CACHE)
endif()

if(ENABLE_KdlSolver)

    # KDLConfig.cmake brings Eigen_INCLUDE_DIR into scope with orocos_kdl_INCLUDE_DIRS
    include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                        ${orocos_kdl_INCLUDE_DIRS})

    yarp_add_plugin(KdlSolver KdlSolver.hpp
                              DeviceDriverImpl.cpp
                              ICartesianSolverImpl.cpp)

    target_link_libraries(KdlSolver YARP::YARP_OS
                                    YARP::YARP_dev
                                    YARP::YARP_sig
                                    ${orocos_kdl_LIBRARIES}
                                    ROBOTICSLAB::ColorDebug
                                    KdlVectorConverterLib
                                    KinematicRepresentationLib
                                    KinematicsDynamicsInterfaces)

    yarp_install(TARGETS KdlSolver
                 COMPONENT runtime
                 LIBRARY DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DYNAMIC_PLUGINS_INSTALL_DIR}
                 ARCHIVE DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_STATIC_PLUGINS_INSTALL_DIR})

    yarp_install(FILES KdlSolver.ini
                 COMPONENT runtime
                 DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
