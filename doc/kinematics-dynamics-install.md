# kinematics-dynamics: Installation from Source Code

First install the dependencies:

- [Install CMake 3.16+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-cmake.md/)
- [Install YCM 0.11+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-ycm.md/)
- [Install YARP 3.9+](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-yarp.md/)
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

## Install Bindings

Swig is needed in order to build all language bindings. Refer to [Install SWIG](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-swig.md/).

### Install Python Bindings

First, install Python development packages.

```bash
sudo apt update
sudo apt install libpython3-dev  # not installed by default on clean distros
```

You can follow these steps after installing kinematics-dynamics, or just activate the correct CMake options during the initial build.

```bash
cd  # go home
cd repos/kinematics-dynamics/build  # this should already exist, see previous section
cmake .. -DCREATE_PYTHON=ON -DCREATE_BINDINGS_PYTHON=ON  # enable Python bindings
make -j  # compile
sudo make install; sudo ldconfig; cd  # install and go home
```

Note: You'll probably want [YARP Python bindings](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-yarp.md/#install-python-bindings) ([perma](https://github.com/roboticslab-uc3m/installation-guides/blob/33c93b68ab34a63157b1dc940dfb154a8504fff8/install-yarp.md#install-python-bindings)), too.

#### Install Python bindings (checking)

Check your installation via (should output nothing; if bad, you will see a `ModuleNotFoundError`):

```bash
python3 -c "import kinematics_dynamics"
```

#### Install Python bindings (troubleshooting)

CMake may not detect the correct Python3 installation directory. Toggle `t` in `ccmake` to see additional configuration. The `CMAKE_INSTALL_PYTHONDIR` variable may point to a wrong path such as `lib/python3/dist-packages` (relative to `CMAKE_INSTALL_PREFIX`, which usually resolves to `/usr/local`). You must pick the python3.x directory instead (check via `python3 -V`); on Ubuntu 20.04 and Python 3.8, this configuration variable should be changed to `lib/python3.8/dist-packages`.

## Even more!

Done! You are now probably interested in one of the following links:
- [Simulation and Basic Control: Now what can I do?]( teo-post-install.md )
