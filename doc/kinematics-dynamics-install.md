## Simulation and Basic Control: Installation from Source Code

First install the dependencies:
- [Install CMake](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install-cmake.md)
- [Install YARP](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install-yarp.md)
- [Install OpenRAVE](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install-openrave.md)
- [Install KDL](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install-kdl.md)

### Install the Simulation and Basic Control Software on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it does not exist; then, enter it
git clone --recursive https://github.com/roboticslab-uc3m/kinematics-dynamics.git  # Download kinematics-dynamics software from the repository; Use --recursive to get embedded repositories (technically, git submodules)
cd kinematics-dynamics; mkdir build; cd build; cmake ..  # Configure the kinematics-dynamics software
make  # Compile
sudo make install  # Install :-)
cp ../scripts/gnome/kinematics-dynamics.desktop $HOME/Desktop
```

For CMake `find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)`, you may also be interested in adding the following to your `~/.bashrc` or `~/.profile`:
```bash
export ROBOTICSLAB_KINEMATICS_DYNAMICS_DIR=$HOME/repos/kinematics-dynamics/build  # Points to where TEOConfig.cmake is generated upon running CMake
```

For additional TEO options use ccmake instead of cmake.

### Even more!

Done! You are now probably interested in one of the following links:
- [Simulation and Basic Control: Now what can I do?]( teo-post-install.md )
