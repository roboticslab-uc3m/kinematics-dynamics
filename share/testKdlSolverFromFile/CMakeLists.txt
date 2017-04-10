# Copyright: 2017 UC3M
# Author: Juan G Victores

set(appname testKdlSolverFromFile)


file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)
#file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template)

yarp_install(FILES ${conf} DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_CONTEXTS_INSTALL_DIR}/${appname})
#yarp_install(FILES ${scripts} DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_APPLICATIONS_TEMPLATES_INSTALL_DIR})

