## Simulation and Basic Control: Installation from Source Code (Ubuntu 14.10)

First install the dependencies:
- [Install CMake (Ubuntu 14.10)](teo_install_cmake_on_ubuntu_14_10)
- [teo_install_yarp_on_ubuntu_14_10](teo_install_yarp_on_ubuntu_14_10.md)
- [teo_install_openrave_on_ubuntu_14_10](teo_install_openrave_on_ubuntu_14_10.md)
- [teo_install_kdl_on_ubuntu_14_10](teo_install_kdl_on_ubuntu_14_10.md)

### Install the Simulation and Basic Control Software

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it doesn't exist; then, enter it
git clone https://github.com/roboticslab-uc3m/teo-main.git  # Download teo-main software from the repository
cd teo-main; mkdir build; cd build; cmake ..  # Configure the teo-main software
make  # Compile
sudo make install  # Install :-)
cp ../scripts/gnome/teo-main.desktop $HOME/Desktop
```

For CMake find_package(TEO REQUIRED), you may also be interested in adding the following to your bashrc or profile:
```bash
export TEO_DIR=/home/teo/repos/teo-main/build
```

For additional TEO options use ccmake instead of cmake.

### Even more!

Done! You are now probably interested in one of the following links:
- teo_post_install
- For the KDL controller, you should look at the \ref KdlBot library.
- teo_environment_variables

