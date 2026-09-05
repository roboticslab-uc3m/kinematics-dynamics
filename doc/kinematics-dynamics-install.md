# kinematics-dynamics: Installation from Source Code

First install the dependencies:

- [Install CMake 3.19+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install YCM 0.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 3.12+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)
- [Install KDL 1.4+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-kdl.md/)

For unit testing, you'll need the googletest source package. Refer to [Install googletest](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-googletest.md/).

## Install kinematics-dynamics on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using `sudo` a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # create $HOME/repos if it does not exist; then, enter it
git clone https://github.com/roboticslab-uc3m/kinematics-dynamics.git  # download kinematics-dynamics sources from GitHub
cd kinematics-dynamics; mkdir build; cd build; cmake ..  # configure the project
make -j  # compile
sudo make install; sudo ldconfig  # install
```

Use `ccmake` instead of `cmake` for additional options.

## Even more!

Done! You are now probably interested in one of the following links:
- [Simulation and Basic Control: Now what can I do?](teo-post-install.md)
