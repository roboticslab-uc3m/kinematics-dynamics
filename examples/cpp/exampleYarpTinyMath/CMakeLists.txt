cmake_minimum_required(VERSION 3.19)

project(exampleYarpTinyMath LANGUAGES CXX)

if(NOT YARP_FOUND)
    find_package(YARP QUIET 3.12 COMPONENTS sig math)

    if(NOT YARP_FOUND)
        find_package(YARP 4.0 REQUIRED COMPONENTS sig math)
    endif()
endif()

if(NOT TARGET ROBOTICSLAB::YarpTinyMathLib)
    find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
endif()

add_executable(exampleYarpTinyMath exampleYarpTinyMath.cpp)

target_link_libraries(exampleYarpTinyMath YARP::YARP_sig
                                          YARP::YARP_math
                                          ROBOTICSLAB::YarpTinyMathLib)
