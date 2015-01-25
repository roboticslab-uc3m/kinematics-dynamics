# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname openrave)

file(GLOB teo ${CMAKE_CURRENT_SOURCE_DIR}/teo/*.xml)
file(GLOB teo/trunk ${CMAKE_CURRENT_SOURCE_DIR}/teo/trunk/*.xml)
file(GLOB teo/rightArm ${CMAKE_CURRENT_SOURCE_DIR}/teo/rightArm/*.xml)

yarp_install(FILES ${teo} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo)
yarp_install(FILES ${teo/trunk} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/trunk)
yarp_install(FILES ${teo/rightArm} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/rightArm)

