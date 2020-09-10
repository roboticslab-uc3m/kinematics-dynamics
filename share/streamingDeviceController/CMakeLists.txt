yarp_install(FILES conf/streamingDeviceController.ini
             DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_CONTEXTS_INSTALL_DIR}/streamingDeviceController)

yarp_install(FILES scripts/streamingSharedControl.xml
             DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_APPLICATIONS_INSTALL_DIR}/streamingDeviceController)
             