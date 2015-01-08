cmake_minimum_required(VERSION 2.8.9)

set(KEYWORD "testColorDebug")

# Start a project.
project(${KEYWORD})

# Search for source code.
file(GLOB folder_source *.cpp *.cc *.c)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# Automatically add include directories if needed.
foreach(header_file ${folder_header})
  get_filename_component(p ${header_file} PATH)
  include_directories(${p})
endforeach(header_file ${folder_header})

# Set up our main executable.
if (folder_source)
  add_executable(${KEYWORD} ${folder_source} ${folder_header})
else (folder_source)
  message(FATAL_ERROR "No source code files found. Please add something")
endif (folder_source)
