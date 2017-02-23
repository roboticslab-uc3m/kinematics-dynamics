# 

cmake_minimum_required (VERSION 2.6.0)

IF (ENABLE_teoCollisionAvoidance)

set(KEYWORD "teoCollisionAvoidance")

# Start a project.
project(${KEYWORD})

#set(CMAKE_BUILD_TYPE Debug)#RelWithDebInfo)
FIND_PACKAGE(OpenRAVE REQUIRED)
FIND_PACKAGE(YARP REQUIRED)

# Search for source code.
file(GLOB folder_source *.cpp *.cc *.c)
file(GLOB folder_header *.h *.hpp)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# Automatically add include directories if needed.
foreach(header_file ${folder_header})
  get_filename_component(p ${header_file} PATH)
  include_directories(${p})
endforeach(header_file ${folder_header})

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

find_package(Boost COMPONENTS iostreams python thread system)

include_directories(${TEO_INCLUDE_DIRS} ${OpenRAVE_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
if( Boost_INCLUDE_DIRS )
  include_directories(${Boost_INCLUDE_DIRS})
endif()

link_directories(${TEO_LINK_DIRS})

# Set up our main executable.
if (folder_source)
  add_executable(${KEYWORD} ${folder_source} ${folder_header})
  set_target_properties(${KEYWORD} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXXFLAGS}")
  set_target_properties(${KEYWORD} PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
  target_link_libraries(${KEYWORD} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${YARP_LIBRARIES})
#  target_link_libraries(${KEYWORD} ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${YARP_LIBRARIES})
  install(TARGETS ${KEYWORD} DESTINATION bin )
else (folder_source)
  message(FATAL_ERROR "No source code files found. Please add something")
endif (folder_source)

ENDIF (ENABLE_teoCollisionAvoidance)

