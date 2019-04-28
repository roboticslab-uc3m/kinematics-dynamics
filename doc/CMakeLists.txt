# https://stackoverflow.com/a/25217937
get_directory_property(_has_parent PARENT_DIRECTORY)

if(_has_parent)
    if(NOT DOXYGEN_FOUND)
        return()
    endif()
else()
    cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
    project(_ NONE) # dummy project, disable compiler check
    find_package(YCM 0.10 REQUIRED)
    find_package(Doxygen REQUIRED)
endif()

set(DOXYGEN_PROJECT_NAME "")
set(DOXYGEN_DOXYGEN_PROJECT_NUMBER "")
set(DOXYGEN_PROJECT_BRIEF "")
set(DOXYGEN_REPEAT_BRIEF NO)
set(DOXYGEN_GROUP_NESTED_COMPOUNDS YES)
set(DOXYGEN_EXTRACT_PRIVATE YES)
set(DOXYGEN_EXTRACT_STATIC YES)
set(DOXYGEN_CITE_BIB_FILES ${CMAKE_CURRENT_SOURCE_DIR}/kinematics-dynamics.bib)
set(DOXYGEN_EXAMPLE_PATH examples)
set(DOXYGEN_EXAMPLE_RECURSIVE YES)
set(DOXYGEN_IMAGE_PATH dox/fig)
set(DOXYGEN_HTML_TIMESTAMP YES)
set(DOXYGEN_FORMULA_FONTSIZE 14)
set(DOXYGEN_HAVE_DOT NO)

set(_doxygen_input doc
                   examples
                   libraries
                   programs
                   share
                   tests)

doxygen_add_docs(dox ${_doxygen_input}
                 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)
