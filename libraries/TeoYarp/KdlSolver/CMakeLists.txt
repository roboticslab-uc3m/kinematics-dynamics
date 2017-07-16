# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Copyright: (C) 2009 RobotCub Consortium
# Author: Juan G. Victores & Raul de Santos Rico
# Contrib: Ugo; Paul Fitzpatrick (YARP device examples and tutorials)
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

yarp_prepare_plugin(KdlSolver
                    CATEGORY device
                    TYPE roboticslab::KdlSolver
                    INCLUDE KdlSolver.hpp
                    DEFAULT ON)

if(NOT SKIP_KdlSolver)

find_package(YARP REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS}
                    ${orocos_kdl_INCLUDE_DIRS})

yarp_add_plugin(KdlSolver KdlSolver.hpp
                          DeviceDriverImpl.cpp
                          ICartesianSolverImpl.cpp)

target_link_libraries(KdlSolver ${ROBOTICSLAB_KINEMATICS_DYNAMICS_LIBRARIES}
                                ${YARP_LIBRARIES}
                                ${orocos_kdl_LIBRARIES})

# Exporting dependencies for TEOConfig.cmake quite manually for now...
set(ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS ${ROBOTICSLAB_KINEMATICS_DYNAMICS_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended libraries")

yarp_install(TARGETS KdlSolver
             EXPORT YARP
             COMPONENT runtime
             LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
             ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})

yarp_install(FILES KdlSolver.ini
             COMPONENT runtime
             DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

endif()
