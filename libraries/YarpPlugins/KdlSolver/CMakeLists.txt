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
                    DEPENDS "ENABLE_KdlVectorConverterLib;ENABLE_KinematicRepresentationLib")

if(NOT SKIP_KdlSolver)

find_package(orocos_kdl REQUIRED)

option(USE_LMA "Use LMA algorithm in KdlSolver" OFF)

# KDLConfig.cmake brings Eigen_INCLUDE_DIR into scope with orocos_kdl_INCLUDE_DIRS
include_directories(${CMAKE_CURRENT_SOURCE_DIR} # yarp plugin builder needs this
                    ${orocos_kdl_INCLUDE_DIRS})

yarp_add_plugin(KdlSolver KdlSolver.hpp
                          DeviceDriverImpl.cpp
                          ICartesianSolverImpl.cpp)

add_dependencies(KdlSolver COLOR_DEBUG)

target_link_libraries(KdlSolver YARP::YARP_OS
                                YARP::YARP_dev
                                YARP::YARP_sig
                                ${orocos_kdl_LIBRARIES}
                                KdlVectorConverterLib
                                KinematicRepresentationLib)

if(USE_LMA)
    target_compile_definitions(KdlSolver PRIVATE _USE_LMA_)
endif()

yarp_install(TARGETS KdlSolver
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES KdlSolver.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
