// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

//-- must be defined BEFORE including ColorDebug.hpp for now.
//#define CD_FULL_FILE  //-- Can be globally managed from father CMake.
//#define CD_HIDE_ERROR  //-- Can be globally managed from father CMake.
//#define CD_HIDE_WARNING  //-- Can be globally managed from father CMake.
//#define CD_HIDE_SUCCESS  //-- Can be globally managed from father CMake.
//#define CD_HIDE_INFO  //-- Can be globally managed from father CMake.
//#define CD_HIDE_DEBUG  //-- Can be globally managed from father CMake.

#include "../ColorDebug.hpp"

int main(int argc, char *argv[]) {

    CD_ERROR("Test CD_ERROR.\n");
    CD_WARNING("Test CD_WARNING.\n");
    CD_SUCCESS("Test CD_SUCCESS.\n");
    CD_INFO("Test CD_INFO.\n");
    CD_DEBUG("Test CD_DEBUG.\n\n");

    CD_ERROR_NO_HEADER("Test CD_ERROR_NO_HEADER.\n");
    CD_WARNING_NO_HEADER("Test CD_WARNING_NO_HEADER.\n");
    CD_SUCCESS_NO_HEADER("Test CD_SUCCESS_NO_HEADER.\n");
    CD_INFO_NO_HEADER("Test CD_INFO_NO_HEADER.\n");
    CD_DEBUG_NO_HEADER("Test CD_DEBUG_NO_HEADER.\n\n");

    return 0;
}
