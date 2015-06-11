# Copyright: 2014 UC3M
# Author: Juan G Victores
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# 

find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(teo)

yarp_install(FILES ymanager.ini DESTINATION ${TEO_DATA_INSTALL_DIR})  # Cleaner than ${TEO_ROBOTS_INSTALL_DIR}/../ or similar.

### Go through single applications
#add_subdirectory(teo)

add_subdirectory(openrave)
add_subdirectory(kinematics)

add_subdirectory(teoGravityCompensator)
add_subdirectory(teoSim)

