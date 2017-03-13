include(YCMEPHelper)

ycm_ep_helper(COLOR_DEBUG TYPE GIT
              STYLE GITHUB
              REPOSITORY roboticslab-uc3m/color-debug.git
              TAG master)

ExternalProject_Get_Property(COLOR_DEBUG INSTALL_DIR)
include_directories(${INSTALL_DIR})
