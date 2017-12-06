## kinematics-dynamics: Installation from Source Code

First install the dependencies:
- [Install CMake](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md)
- [Install YARP](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md)
- [Install KDL](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-kdl.md)
- Only for `testBasicCartesianControl`, we use `FakeControlboard` from [openrave-yarp-plugins](https://github.com/roboticslab-uc3m/openrave-yarp-plugins):
```bash
cd  # go home
mkdir -p repos; cd repos  # create $HOME/repos if it does not exist; then, enter it
git clone https://github.com/roboticslab-uc3m/openrave-yarp-plugins
cd openrave-yarp-plugins
git checkout 8b4a803b6f3443921fb8d8b9c28b723a06514529
mkdir build && cd build
cmake .. -DENABLE_OpenraveYarpControlboard=OFF -DENABLE_OpenraveYarpPaintSquares=OFF -DENABLE_OpenraveWorldRpcResponder=OFF -DENABLE_OpenraveYarpForceEstimator=OFF -DENABLE_OpenraveYarpPluginLoader=OFF -DENABLE_YarpOpenraveBase=OFF -DENABLE_YarpOpenraveControlboard=OFF -DENABLE_YarpOpenraveControlboardCollision=OFF -DENABLE_YarpOpenraveGrabber=OFF -DENABLE_YarpOpenraveRGBDSensor=OFF -DENABLE_YarpOpenraveRobotManager=OFF -DENABLE_OpenraveYarpCoupled=OFF -DENABLE_teoSim=OFF -DENABLE_FakeControlboard=ON
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
