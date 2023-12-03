if(ENABLE_ftCompensation)
    yarp_install(FILES applications/ftCompensation.xml
                 DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_APPLICATIONS_INSTALL_DIR}/ftCompensation)

    yarp_install(FILES contexts/ftCompensation-lacquey.ini
                       contexts/ftCompensation-stump.ini
                 DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_CONTEXTS_INSTALL_DIR}/ftCompensation)
endif()

if(ENABLE_streamingDeviceController)
    yarp_install(FILES contexts/streamingDeviceController.ini
                 DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_CONTEXTS_INSTALL_DIR}/streamingDeviceController)
endif()

if(ENABLE_tests AND ENABLE_AsibotSolver)
    yarp_install(FILES contexts/testAsibotSolverFromFile.ini
                 DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_CONTEXTS_INSTALL_DIR}/testAsibotSolverFromFile)
endif()

if(ENABLE_tests AND ENABLE_KdlSolver)
    yarp_install(FILES contexts/testKdlSolverFromFile.ini
                 DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_CONTEXTS_INSTALL_DIR}/testKdlSolverFromFile)
endif()

yarp_install(FILES applications/ymanager.ini
             DESTINATION ${ROBOTICSLAB-KINEMATICS-DYNAMICS_APPLICATIONS_INSTALL_DIR})
