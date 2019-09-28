[![Teo-main Homepage](https://img.shields.io/badge/kinematics-dynamics-orange.svg)](http://robots.uc3m.es/dox-kinematics-dynamics)

Kinematics and dynamics solvers and controllers.

Link to Doxygen generated documentation: http://robots.uc3m.es/dox-kinematics-dynamics

<p align="center">
    <img src="doc/fig/kinematics-dynamics.png" alt="kinematics-dynamics image"/>
</p>

## Installation

Installation instructions for installing from source can be found [here](doc/kinematics-dynamics-install.md).

## Contributing

#### Posting Issues

1. Read [CONTRIBUTING.md](https://github.com/roboticslab-uc3m/kinematics-dynamics/blob/master/CONTRIBUTING.md)
2. [Post an issue / Feature request / Specific documentation request](https://github.com/roboticslab-uc3m/kinematics-dynamics/issues)

#### Fork & Pull Request

1. [Fork the repository](https://github.com/roboticslab-uc3m/kinematics-dynamics/fork)
2. Create your feature branch (`git checkout -b my-new-feature`) off the `develop` branch, following the [GitFlow git workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow)
3. Commit your changes
4. Push to the branch (`git push origin my-new-feature`)
5. Create a new Pull Request

## Status

[![Build Status (Linux)](https://travis-ci.com/roboticslab-uc3m/kinematics-dynamics.svg?branch=develop)](https://travis-ci.com/roboticslab-uc3m/kinematics-dynamics)

[![Coverage Status](https://coveralls.io/repos/roboticslab-uc3m/kinematics-dynamics/badge.svg)](https://coveralls.io/r/roboticslab-uc3m/kinematics-dynamics)

[![Issues](https://img.shields.io/github/issues/roboticslab-uc3m/kinematics-dynamics.svg?label=Issues)](https://github.com/roboticslab-uc3m/kinematics-dynamics/issues)

## Similar and Related Projects

### Fast Solvers
- [ocra-recipes/eigen_lgsm](https://github.com/ocra-recipes/eigen_lgsm): used by [robotology/codyco-superbuild](https://github.com/robotology/codyco-superbuild)
- [cuSolver](https://docs.nvidia.com/cuda/cusolver/index.html)

### Fast IK-Solvers
- [IKFast](http://openrave.org/docs/0.8.2/ikfast/): Part of [OpenRAVE](http://openrave.org/) ([rdiankov/openrave](https://github.com/rdiankov/openrave), [roboticslab-uc3m/installation-guides](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-openrave.md))
- [NUKE](https://vanadiumlabs.github.io/pypose/nuke-intro.html#NUKE): The Nearly Universal Kinematic Engine
- [ESROCOS/kin-gen](https://github.com/ESROCOS/kin-gen): Kinematics code generator by KUL
- [AversivePlusPlus/ik](https://github.com/AversivePlusPlus/ik)
- [ros-industrial-consortium/descartes](https://github.com/ros-industrial-consortium/descartes)

### Kinematics and Dynamics
- [orocos/orocos_kinematics_dynamics](https://github.com/orocos/orocos_kinematics_dynamics) ([roboticslab-uc3m/installation-guides](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-kdl.md)): A dependency of this repository
- [iDyn](http://www.icub.org/doc/icub-main/idyn_introduction.html): Library in [robotology/icub-main](https://github.com/robotology/icub-main) for computing kinematics and dynamics of serial-links chains of revolute joints and limbs

### Path-Planning, Trajectory generation and optimization
- All the parts of [OpenRAVE](http://openrave.org/) ([rdiankov/openrave](https://github.com/rdiankov/openrave), [roboticslab-uc3m/installation-guides](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-openrave.md)) we do not use
- [ros-industrial-consortium/trajopt\_ros](https://github.com/ros-industrial-consortium/trajopt_ros): Trajectory Optimization Motion Planner for ROS
- https://rosindustrial.org/news/2018/7/5/optimization-motion-planning-with-tesseract-and-trajopt-for-industrial-applications
- [ROSPlan](http://kcl-planning.github.io/ROSPlan/) ([KCL-Planning/ROSPlan](https://github.com/KCL-Planning/ROSPlan)): Tools for AI Planning in a ROS system.
- [jrl-umi3218/Tasks](https://github.com/jrl-umi3218/Tasks): It has been used extensively to control humanoid robots such as HOAP-3, HRP-2, HRP-4 and Atlas.
- [googlecartographer (org)](https://github.com/googlecartographer): Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configuration

### Humanoid-oriented
- [roboticslab-uc3m/gait](https://github.com/roboticslab-uc3m/gait)
- [roboticslab-uc3m/gaitcontrol](https://github.com/roboticslab-uc3m/gaitcontrol)
- [roboticslab-uc3m/footsteps](https://github.com/roboticslab-uc3m/footsteps): Includes interesting links
- [robotology/walking-controllers](https://github.com/robotology/walking-controllers)
- [robotology/whole-body-controllers](https://github.com/robotology/whole-body-controllers)
- [stephane-caron/pymanoid](https://github.com/stephane-caron/pymanoid): Humanoid robotics prototyping environment based on [OpenRAVE](http://openrave.org/) ([rdiankov/openrave](https://github.com/rdiankov/openrave), [roboticslab-uc3m/installation-guides](https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-openrave.md))
- [AIS-Bonn/humanoid_op_ros](https://github.com/AIS-Bonn/humanoid_op_ros): Contains interesting walking motion in [./src/nimbro/motion](https://github.com/AIS-Bonn/humanoid_op_ros/tree/master/src/nimbro/motion)
- [Stack of Tasks](https://stack-of-tasks.github.io/) ([stack-of-tasks (org)](https://github.com/stack-of-tasks))
- [nav74neet/ddpg_biped](https://github.com/nav74neet/ddpg_biped)
