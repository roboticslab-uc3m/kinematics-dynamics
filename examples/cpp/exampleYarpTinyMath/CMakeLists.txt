cmake_minimum_required(VERSION 3.12)

project(exampleYarpTinyMath LANGUAGES CXX)

if(NOT YARP_math_FOUND)
    find_package(YARP 3.5 REQUIRED COMPONENTS sig math)
endif()

if(NOT TARGET ROBOTICSLAB::YarpTinyMathLib)
    find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
endif()

add_executable(exampleYarpTinyMath exampleYarpTinyMath.cpp)

target_link_libraries(exampleYarpTinyMath YARP::YARP_sig
                                          YARP::YARP_math
                                          ROBOTICSLAB::YarpTinyMathLib)
