option(ENABLE_ftCompensation "Enable/disable ftCompensation program" ON)

if(ENABLE_ftCompensation)

    add_executable(ftCompensation main.cpp
                                  FtCompensation.hpp
                                  FtCompensation.cpp
                                  LogComponent.hpp
                                  LogComponent.cpp)

    target_link_libraries(ftCompensation YARP::YARP_os
                                         YARP::YARP_init
                                         YARP::YARP_dev)

    install(TARGETS ftCompensation
            DESTINATION ${CMAKE_INSTALL_BINDIR})

endif()
