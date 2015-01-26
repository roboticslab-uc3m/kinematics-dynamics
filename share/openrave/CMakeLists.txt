# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname openrave)

file(GLOB teo ${CMAKE_CURRENT_SOURCE_DIR}/teo/*.xml)
file(GLOB teo/trunk/xml ${CMAKE_CURRENT_SOURCE_DIR}/teo/trunk/*.xml)
file(GLOB teo/trunk/wrl ${CMAKE_CURRENT_SOURCE_DIR}/teo/trunk/*.wrl)
file(GLOB teo/leftArm/xml ${CMAKE_CURRENT_SOURCE_DIR}/teo/leftArm/*.xml)
file(GLOB teo/leftArm/wrl ${CMAKE_CURRENT_SOURCE_DIR}/teo/leftArm/*.wrl)
file(GLOB teo/rightArm/xml ${CMAKE_CURRENT_SOURCE_DIR}/teo/rightArm/*.xml)
file(GLOB teo/rightArm/wrl ${CMAKE_CURRENT_SOURCE_DIR}/teo/rightArm/*.wrl)
file(GLOB teo/leftLeg/xml ${CMAKE_CURRENT_SOURCE_DIR}/teo/leftLeg/*.xml)
file(GLOB teo/leftLeg/wrl ${CMAKE_CURRENT_SOURCE_DIR}/teo/leftLeg/*.wrl)

yarp_install(FILES ${teo} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo)
yarp_install(FILES ${teo/trunk/xml} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/trunk)
yarp_install(FILES ${teo/trunk/wrl} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/trunk)
yarp_install(FILES ${teo/leftArm/xml} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/leftArm)
yarp_install(FILES ${teo/leftArm/wrl} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/leftArm)
yarp_install(FILES ${teo/rightArm/xml} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/rightArm)
yarp_install(FILES ${teo/rightArm/wrl} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/rightArm)
yarp_install(FILES ${teo/rightLeg/xml} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/rightLeg)
yarp_install(FILES ${teo/rightLeg/wrl} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/rightLeg)

