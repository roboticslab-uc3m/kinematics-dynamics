## kinematics-dynamics: Installation from Source Code

First install the dependencies:

- [Install CMake](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md)
- [Install YARP](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md)
- [Install KDL](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-kdl.md)

Additionally, this project depends on YCM to download and build external packages. Although this process is intended to run automatically during the CMake configuration phase, you may still want to install YCM and said packages by yourself. In that respect, refer to [Install YCM](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md) and to the installation guides of any package listed below:

- [color-debug](https://github.com/roboticslab-uc3m/color-debug)

Only for `testBasicCartesianControl`, we use `FakeControlboard` from [yarp-devices](https://github.com/roboticslab-uc3m/yarp-devices):

```bash
cd  # go home
mkdir -p repos; cd repos  # create $HOME/repos if it does not exist; then, enter it
git clone https://github.com/roboticslab-uc3m/yarp-devices
cd yarp-devices
git checkout 5ffad0e616aa74549c7aa1dda7e4b43078ac703e
mkdir build && cd build
cmake .. -DENABLE_tests=OFF -DENABLE_checkCanBus=OFF -DENABLE_dumpCanBus=OFF -DENABLE_OneCanBusOneWrapper=OFF -DENABLE_TwoCanBusThreeWrappers=OFF -DENABLE_CanBusControlBoard=OFF -DENABLE_CanBusHico=OFF -DENABLE_CuiAbsolute=OFF -DENABLE_FakeJoint=OFF -DENABLE_Jr3=OFF -DENABLE_LacqueyFetch=OFF -DENABLE_LeapMotionSensor=OFF -DENABLE_ProximitySensorsClient=OFF -DENABLE_SpaceNavigator=OFF -DENABLE_TechnosoftIpos=OFF -DENABLE_TextilesHand=OFF -DENABLE_WiimoteSensor=OFF
make -j$(nproc)  # compile
sudo make install
cd ../..
```

### Install kinematics-dynamics on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using `sudo` a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # create $HOME/repos if it does not exist; then, enter it
git clone --recursive https://github.com/roboticslab-uc3m/kinematics-dynamics.git  # Download kinematics-dynamics software from the repository; Use --recursive to get embedded repositories (technically, git submodules)
cd kinematics-dynamics; mkdir build; cd build; cmake ..  # Configure the kinematics-dynamics software
make -j$(nproc) # Compile
sudo make install  # Install :-)
```

For CMake `find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)`, you may also be interested in adding the following to your `~/.bashrc` or `~/.profile`:
```bash
export ROBOTICSLAB_KINEMATICS_DYNAMICS_DIR=$HOME/repos/kinematics-dynamics/build  # Points to where TEOConfig.cmake is generated upon running CMake
```

For additional options use ccmake instead of cmake.

### Even more!

Done! You are now probably interested in one of the following links:
- [Simulation and Basic Control: Now what can I do?]( teo-post-install.md )
