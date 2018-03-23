# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

yarp_install(FILES ymanager.ini
             DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_DATA_INSTALL_DIR})

### Go through single applications
add_subdirectory(streamingDeviceController)
add_subdirectory(testAsibotSolverFromFile)
add_subdirectory(testKdlSolverFromFile)
