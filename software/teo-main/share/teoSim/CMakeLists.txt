# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

set(appname teoSim)

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
#file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

file(GLOB models/teoWhite ${CMAKE_CURRENT_SOURCE_DIR}/models/teoWhite/*.xml)
file(GLOB models/teoWhite/wrls/chest ${CMAKE_CURRENT_SOURCE_DIR}/models/teoWhite/wrls/chest/*.wrl)
file(GLOB models/teoWhite/wrls/head ${CMAKE_CURRENT_SOURCE_DIR}/models/teoWhite/wrls/head/*.wrl)
file(GLOB models/teoWhite/wrls/hip ${CMAKE_CURRENT_SOURCE_DIR}/models/teoWhite/wrls/hip/*.wrl)
file(GLOB models/teoWhite/wrls/left_arm ${CMAKE_CURRENT_SOURCE_DIR}/models/teoWhite/wrls/left_arm/*.wrl)
file(GLOB models/teoWhite/wrls/left_leg ${CMAKE_CURRENT_SOURCE_DIR}/models/teoWhite/wrls/left_leg/*.wrl)
file(GLOB models/teoWhite/wrls/right_arm ${CMAKE_CURRENT_SOURCE_DIR}/models/teoWhite/wrls/right_arm/*.wrl)
file(GLOB models/teoWhite/wrls/right_leg ${CMAKE_CURRENT_SOURCE_DIR}/models/teoWhite/wrls/right_leg/*.wrl)


yarp_install(FILES ${conf} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname})
#yarp_install(FILES ${scripts} DESTINATION ${TEO_APPLICATIONS_TEMPLATES_INSTALL_DIR})

yarp_install(FILES ${models/teoWhite} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/models/teoWhite)
yarp_install(FILES ${models/teoWhite/wrls/chest} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/models/teoWhite/wrls/chest)
yarp_install(FILES ${models/teoWhite/wrls/head} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/models/teoWhite/wrls/head)
yarp_install(FILES ${models/teoWhite/wrls/hip} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/models/teoWhite/wrls/hip)
yarp_install(FILES ${models/teoWhite/wrls/left_arm} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/models/teoWhite/wrls/left_arm)
yarp_install(FILES ${models/teoWhite/wrls/left_leg} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/models/teoWhite/wrls/left_leg)
yarp_install(FILES ${models/teoWhite/wrls/right_arm} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/models/teoWhite/wrls/right_arm)
yarp_install(FILES ${models/teoWhite/wrls/right_leg} DESTINATION ${TEO_CONTEXTS_INSTALL_DIR}/${appname}/models/teoWhite/wrls/right_leg)

