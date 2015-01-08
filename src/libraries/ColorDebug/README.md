ColorDebug
==========

Color CLI logs and more.

See your version in ColorDebug.hpp along with license and author information.

Usage for now: copy this directory, and add the following two lines to the parent's CMakeLists.txt

set(COLOR_DEBUG_PART_OF_PROJECT TRUE)

add_subdirectory(ColorDebug)  # ColorDebug sets COLOR_DEBUG_INCLUDE_DIRS

