# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname openrave)

file(GLOB teo ${CMAKE_CURRENT_SOURCE_DIR}/teo/*.xml)
file(GLOB teo/models ${CMAKE_CURRENT_SOURCE_DIR}/teo/models/*.wrl)

yarp_install(FILES ${teo} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo)
yarp_install(FILES ${teo/models} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/teo/models)

