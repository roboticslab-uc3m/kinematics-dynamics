[![Teo-main Homepage](https://img.shields.io/badge/kinematics-dynamics-orange.svg)](https://robots.uc3m.es/kinematics-dynamics/)

Kinematics and dynamics solvers and controllers.

Link to Doxygen generated documentation: https://robots.uc3m.es/kinematics-dynamics/

<p align="center"><img src="https://raw.githubusercontent.com/roboticslab-uc3m/kinematics-dynamics/master/doc/fig/kinematics-dynamics.png" alt="kinematics-dynamics image"/></p>

## Installation

Installation instructions for installing from source can be found [here](doc/kinematics-dynamics-install.md).

## Contributing

#### Posting Issues

1. Read [CONTRIBUTING.md](CONTRIBUTING.md)
2. [Post an issue / Feature request / Specific documentation request](https://github.com/roboticslab-uc3m/kinematics-dynamics/issues)

#### Fork & Pull Request

1. [Fork the repository](https://github.com/roboticslab-uc3m/kinematics-dynamics/fork)
2. Create your feature branch (`git checkout -b my-new-feature`) off the `master` branch, following the [Forking Git workflow](https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow)
3. Commit your changes
4. Push to the branch (`git push origin my-new-feature`)
5. Create a new Pull Request

## Citation

If you found this project useful, please consider citing the following works:

- [ScrewTheoryLib](libraries/ScrewTheoryLib/)

Bartek Lukawski, Ignacio Montesino Valle, Juan G. Victores, Alberto Jardón, and Carlos Balaguer. An inverse kinematics problem solver based on screw theory for manipulator arms. In *XLIII Jornadas de Automática*, pages 864–869. Universidade da Coruña, 2022.

```bibtex
@inproceedings{lukawski2022jjaa,
    author    = {{\L}ukawski, Bartek and Montesino Valle, Ignacio and Victores, Juan G. and Jardón, Alberto and Balaguer, Carlos},
    title     = {An inverse kinematics problem solver based on screw theory for manipulator arms},
    booktitle = {XLIII Jornadas de Automática},
    year      = {2022},
    pages     = {864--869},
    publisher = {Universidade da Coruña},
    doi       = {10.17979/spudc.9788497498418.0864},
}
```

- [streamingDeviceController](programs/streamingDeviceController/)

Edwin Daniel Oña, Bartek Lukawski, Alberto Jardón, and Carlos Balaguer. A modular framework to facilitate the control of an assistive robotic arm using visual servoing and proximity sensing. In *IEEE Int. Conf. on Autonomous Robot Systems and Competitions (ICARSC)*, pages 28–33, 2020.

```bibtex
@inproceedings{eona2020icarsc,
    author    = {{O\~na}, Edwin Daniel and {\L}ukawski, Bartek and Jardón, Alberto and Balaguer, Carlos},
    title     = {A modular framework to facilitate the control of an assistive robotic arm using visual servoing and proximity sensing},
    booktitle = {IEEE Int. Conf. on Autonomous Robot Systems and Competitions (ICARSC)},
    year      = {2020},
    pages     = {28--33},
    doi       = {10.1109/ICARSC49921.2020.9096146},
}
```

Bartek Lukawski, Juan G. Victores, and Carlos Balaguer. A generic controller for teleoperation on robotic manipulators using low-cost devices. In *XLIV Jornadas de Automática*, pages 785–788. Universidade da Coruña, 2023.

```bibtex
@inproceedings{lukawski2023jjaa,
    author    = {{\L}ukawski, Bartek and Victores, Juan G. and Balaguer, Carlos},
    title     = {A generic controller for teleoperation on robotic manipulators using low-cost devices},
    booktitle = {XLIV Jornadas de Automática},
    year      = {2023},
    pages     = {785--788},
    publisher = {Universidade da Coruña},
    doi       = {10.17979/spudc.9788497498609.785},
}
```

## Status

[![CI (Linux)](https://github.com/roboticslab-uc3m/kinematics-dynamics/workflows/Continuous%20Integration/badge.svg)](https://github.com/roboticslab-uc3m/kinematics-dynamics/actions)

[![Coverage Status](https://coveralls.io/repos/roboticslab-uc3m/kinematics-dynamics/badge.svg)](https://coveralls.io/r/roboticslab-uc3m/kinematics-dynamics)

[![Issues](https://img.shields.io/github/issues/roboticslab-uc3m/kinematics-dynamics.svg?label=Issues)](https://github.com/roboticslab-uc3m/kinematics-dynamics/issues)

## Similar and Related Projects

### Quaternions

- [pyquaternion](http://kieranwynn.github.io/pyquaternion/) ([KieranWynn/pyquaternion](https://github.com/KieranWynn/pyquaternion))

### Fast Solvers

- [ocra-recipes/eigen_lgsm](https://github.com/ocra-recipes/eigen_lgsm): used by [robotology/codyco-superbuild](https://github.com/robotology/codyco-superbuild)
- [cuSolver](https://docs.nvidia.com/cuda/cusolver/index.html)

### IK-Solvers

- [IKFast](http://openrave.org/docs/0.8.2/ikfast/): Part of [OpenRAVE](http://openrave.org/) ([rdiankov/openrave](https://github.com/rdiankov/openrave), [roboticslab-uc3m/installation-guides](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-openrave.md))
- [NUKE](https://vanadiumlabs.github.io/pypose/nuke-intro.html#NUKE): The Nearly Universal Kinematic Engine
- [ESROCOS/kin-gen](https://github.com/ESROCOS/kin-gen): Kinematics code generator by KUL
- [AversivePlusPlus/ik](https://github.com/AversivePlusPlus/ik)
- [ros-industrial-consortium/descartes](https://github.com/ros-industrial-consortium/descartes)
- [IKPy](https://phylliade.github.io/ikpy) ([Phylliade/ikpy](https://github.com/Phylliade/ikpy))
- [uts-magic-lab/Magiks](https://github.com/uts-magic-lab/Magiks)
- [tasts-robots/pink](https://github.com/tasts-robots/pink): Based on Pinocchio

### Kinematics and Dynamics

- [orocos/orocos_kinematics_dynamics](https://github.com/orocos/orocos_kinematics_dynamics) ([roboticslab-uc3m/installation-guides](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-kdl.md)): A dependency of this repository
- [iDyn](http://www.icub.org/doc/icub-main/idyn_introduction.html): Library in [robotology/icub-main](https://github.com/robotology/icub-main) for computing kinematics and dynamics of serial-links chains of revolute joints and limbs
- [stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [RBDL](https://rbdl.github.io/) ([rbdl/rbdl](https://github.com/rbdl/rbdl)): Rigid Body Dynamics Library. The code tightly follows the notation used in Roy Featherstone's book "Rigid Body Dynamics Algorithm".
- [adityadua24/robopy](https://github.com/adityadua24/robopy)
- [jdj2261/pykin](https://github.com/jdj2261/pykin)

### Path-Planning, Trajectory generation and optimization

- All the parts of [OpenRAVE](http://openrave.org/) ([rdiankov/openrave](https://github.com/rdiankov/openrave), [roboticslab-uc3m/installation-guides](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-openrave.md)) we do not use
- [PythonRobotics](https://atsushisakai.github.io/PythonRobotics/) ([AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics))
- [ros-industrial-consortium/trajopt\_ros](https://github.com/ros-industrial-consortium/trajopt_ros): Trajectory Optimization Motion Planner for ROS (uses http://rll.berkeley.edu/trajopt)
- [pantor/ruckig](https://github.com/pantor/ruckig): Online Trajectory Generation. Real-time. Time-optimal. Jerk-constrained.
- https://rosindustrial.org/news/2018/7/5/optimization-motion-planning-with-tesseract-and-trajopt-for-industrial-applications
- [ROSPlan](http://kcl-planning.github.io/ROSPlan/) ([KCL-Planning/ROSPlan](https://github.com/KCL-Planning/ROSPlan)): Tools for AI Planning in a ROS system.
- [jrl-umi3218/Tasks](https://github.com/jrl-umi3218/Tasks): It has been used extensively to control humanoid robots such as HOAP-3, HRP-2, HRP-4 and Atlas.
- [googlecartographer (org)](https://github.com/googlecartographer): Cartographer is a system that provides real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configuration

### Humanoid-oriented

- [roboticslab-uc3m/gait](https://github.com/roboticslab-uc3m/gait)
- [roboticslab-uc3m/gaitcontrol](https://github.com/roboticslab-uc3m/gaitcontrol)
- [roboticslab-uc3m/TEOTraGen](https://github.com/roboticslab-uc3m/TEOTraGen)
- [roboticslab-uc3m/footsteps](https://github.com/roboticslab-uc3m/footsteps): Includes interesting links
- [munozyanez/spgait](https://github.com/munozyanez/spgait)
- [robotology](https://github.com/robotology)
  - [robotology/walking-controllers](https://github.com/robotology/walking-controllers)
  - [robotology/whole-body-controllers](https://github.com/robotology/whole-body-controllers)
- [epfl-lasa/icub-ds-walking](https://github.com/epfl-lasa/icub-ds-walking)
- [stephane-caron](https://github.com/stephane-caron)
  - [stephane-caron/lipm_walking_controller](https://github.com/stephane-caron/lipm_walking_controller) ([wiki](https://github.com/stephane-caron/lipm_walking_controller/wiki/How-to-use-the-graphical-user-interface%3F), [docker](https://hub.docker.com/r/stephanecaron/lipm_walking_controller))
  - [stephane-caron/pymanoid](https://github.com/stephane-caron/pymanoid): Humanoid robotics prototyping environment based on [OpenRAVE](http://openrave.org/) ([rdiankov/openrave](https://github.com/rdiankov/openrave), [roboticslab-uc3m/installation-guides](https://github.com/roboticslab-uc3m/installation-guides/blob/master/docs/install-openrave.md))
- [Stack of Tasks](https://stack-of-tasks.github.io/) ([stack-of-tasks (org)](https://github.com/stack-of-tasks))
- [Humanoid Path Planner](https://humanoid-path-planner.github.io/hpp-doc) ([humanoid-path-planner (org)](https://github.com/humanoid-path-planner))
- [AIS-Bonn/humanoid_op_ros](https://github.com/AIS-Bonn/humanoid_op_ros): Contains interesting walking motion in [./src/nimbro/motion](https://github.com/AIS-Bonn/humanoid_op_ros/tree/master/src/nimbro/motion)
- [adamlukomski/iva](https://github.com/adamlukomski/iva)
- [pal-robotics](https://github.com/pal-robotics)
- [loco-3d](https://github.com/loco-3d)
- https://discourse.ros.org/t/humanoids-sig/1949/12
- [isri-aist](https://github.com/isri-aist)
  - [isri-aist/BaselineWalkingController](https://github.com/isri-aist/BaselineWalkingController) ([docker](https://github.com/orgs/isri-aist/packages?repo_name=BaselineWalkingController))
  - [isri-aist/CentroidalControlCollection](https://github.com/isri-aist/CentroidalControlCollection)
- via learning
  - [DLR-RM/rl-baselines3-zoo](https://github.com/DLR-RM/rl-baselines3-zoo) includes humanoid (also see pretrained at <https://huggingface.co/sb3/tqc-Humanoid-v3>)
  - <https://es.mathworks.com/help/sm/ug/humanoid_walker.html>
  - [nav74neet/ddpg_biped](https://github.com/nav74neet/ddpg_biped)
