option(ENABLE_examples "Enable/disable C++ examples" OFF)

if(ENABLE_examples)
    if(TARGET ROBOTICSLAB::KinematicsDynamicsInterfaces)
        add_subdirectory(exampleCartesianControlClient)
    endif()

    if(TARGET ROBOTICSLAB::KdlVectorConverterLib
            AND TARGET ROBOTICSLAB::KinematicRepresentationLib
            AND TARGET ROBOTICSLAB::TrajectoryLib
            AND TARGET ROBOTICSLAB::ScrewTheoryLib)
        add_subdirectory(exampleScrewTheoryTrajectory)
    endif()

    if(TARGET ROBOTICSLAB::YarpTinyMathLib)
        add_subdirectory(exampleYarpTinyMath)
    endif()
endif()
