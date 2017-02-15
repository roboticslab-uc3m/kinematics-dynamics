## Simulation and Basic Control: Installation from Source Code

First install the dependencies:
- [Install CMake](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_cmake.md)
- [Install YARP](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_yarp.md)
- [Install OpenRAVE](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_openrave.md)
- [Install KDL](https://github.com/roboticslab-uc3m/installation-guides/blob/develop/install_kdl.md)

### Install the Simulation and Basic Control Software on Ubuntu (working on all tested versions)

Our software integrates the previous dependencies. Note that you will be prompted for your password upon using '''sudo''' a couple of times:

```bash
cd  # go home
mkdir -p repos; cd repos  # make $HOME/repos if it does not exist; then, enter it
git clone --recursive https://github.com/roboticslab-uc3m/teo-main.git  # Download teo-main software from the repository; Use --recursive to get embedded repositories (technically, git submodules)
cd teo-main; mkdir build; cd build; cmake ..  # Configure the teo-main software
make  # Compile
sudo make install  # Install :-)
cp ../scripts/gnome/teo-main.desktop $HOME/Desktop
```

For CMake `find_package(TEO REQUIRED)`, you may also be interested in adding the following to your `~/.bashrc` or `~/.profile`:
```bash
export TEO_DIR=$HOME/repos/teo-main/build
```

For additional TEO options use ccmake instead of cmake.

### Even more!

Done! You are now probably interested in one of the following links:
- [Simulation and Basic Control: Now what can I do?]( teo_post_install.md )
- For the KDL controller, you should look at the \ref KdlBot library.

