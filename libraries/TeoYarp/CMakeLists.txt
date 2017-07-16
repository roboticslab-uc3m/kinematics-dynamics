# Copyright: (C) 2013 Universidad Carlos III de Madrid
# Authors: Juan G. Victores

find_package(YARP REQUIRED)
include_directories(${YARP_INCLUDE_DIRS})
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpPlugin)

add_subdirectory(BasicCartesianControl)
add_subdirectory(BasicTwoLimbCartesianControl)
add_subdirectory(CartesianControlClient)
add_subdirectory(CartesianControlServer)
add_subdirectory(KdlSolver)
add_subdirectory(TwoLimbCartesianControlServer)

install(FILES ICartesianControl.h
              ICartesianSolver.h
              ITwoLimbCartesianControl.h
        DESTINATION include)
