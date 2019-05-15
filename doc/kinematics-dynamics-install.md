## kinematics-dynamics: Installation from Source Code

First install the dependencies:

- [Install CMake 3.5+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install YCM 0.10+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 3.0+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)
- [Install KDL 1.4+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-kdl.md/)
- [Install color-debug](https://github.com/roboticslab-uc3m/color-debug)

Only for `testBasicCartesianControl` and `streamingDeviceController`, we use `FakeControlboard` and `ProximitySensorsClient` from [yarp-devices](https://github.com/roboticslab-uc3m/yarp-devices), respectively:

```bash
cd  # go home
mkdir -p repos; cd repos  # create $HOME/repos if it does not exist; then, enter it
git clone https://github.com/roboticslab-uc3m/yarp-devices
cd yarp-devices
mkdir build && cd build
cmake .. -DENABLE_OneCanBusOneWrapper=OFF -DENABLE_TwoCanBusThreeWrappers=OFF -DENABLE_dumpCanBus=OFF -DENABLE_checkCanBus=OFF -DENABLE_oneCanBusOneWrapper=OFF -DENABLE_launchManipulation=OFF -DENABLE_launchLocomotion=OFF -DENABLE_CanBusControlboard=OFF -DENABLE_CanBusHico=OFF -DENABLE_CuiAbsolute=OFF -DENABLE_FakeControlboard=ON -DENABLE_FakeJoint=OFF -DENABLE_Jr3=OFF -DENABLE_LacqueyFetch=OFF -DENABLE_LeapMotionSensor=OFF -DENABLE_ProximitySensorsClient=ON -DENABLE_SpaceNavigator=OFF -DENABLE_TechnosoftIpos=OFF -DENABLE_TextilesHand=OFF -DENABLE_WiimoteSensor=OFF -DENABLE_tests=OFF
make -j$(nproc)  # compile
sudo make install
cd ../..
```

For unit testing, you'll need the googletest source package. Refer to [Install googletest](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-googletest.md/).

### Install kinematics-dynamics on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using `sudo` a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # create $HOME/repos if it does not exist; then, enter it
git clone https://github.com/roboticslab-uc3m/kinematics-dynamics.git  # Download kinematics-dynamics software from the repository
cd kinematics-dynamics; mkdir build; cd build; cmake ..  # Configure the kinematics-dynamics software
make -j$(nproc) # Compile
sudo make install  # Install :-)
```

For CMake `find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)`, you may also be interested in adding the following to your `~/.bashrc` or `~/.profile`:
```bash
export ROBOTICSLAB_KINEMATICS_DYNAMICS_DIR=$HOME/repos/kinematics-dynamics/build  # Points to where TEOConfig.cmake is generated upon running CMake
```

For additional options use `ccmake` instead of `cmake`.

# Install Bindings

Swig is needed in order to build all language bindings. Refer to [Install SWIG](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-swig.md/).

## Install Python bindings

First, install Python development packages.

```bash
sudo apt update
sudo apt install libpython-dev  # not installed by default on clean distros
```

Make sure you have previously installed `kinematics-dynamics`.

```bash
cd  # go home
cd repos/kinematics-dynamics/bindings
mkdir build && cd build
cmake .. -DCREATE_PYTHON=ON
make -j$(nproc)  # compile
sudo make install; sudo ldconfig; cd # install and go home
```

Also, extra care should be taken with Python 2 vs 3 (e.g. toggle `t` to see paths 2.7 vs 3.5m). You may have to:
```bash
sudo ln -s /usr/local/lib/python3/dist-packages/_kinematics_dynamics.so /usr/local/lib/python3.5/dist-packages/
sudo ln -s /usr/local/lib/python3/dist-packages/kinematics_dynamics.py /usr/local/lib/python3.5/dist-packages/
```

### Even more!

Done! You are now probably interested in one of the following links:
- [Simulation and Basic Control: Now what can I do?]( teo-post-install.md )
