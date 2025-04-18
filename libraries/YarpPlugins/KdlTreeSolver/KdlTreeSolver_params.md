| Group |  Parameter |      Type      | Units |  Default Value | Required |                    Description                    |    Notes     |
|:-----:|:----------:|:--------------:|:-----:|:--------------:|:--------:|:-------------------------------------------------:|:------------:|
|       | kinematics |     string     |       |                |    no    | path to file with description of robot kinematics |              |
|       |   gravity  | vector<double> | m/s^2 | (0.0 0.0 9.81) |    no    |                   gravity vector                  |              |
|       |    ikPos   |     string     |       |      nrjl      |    no    |           IK position solver algorithm            | nrjl, online |
|       |   epsPos   |     double     |   m   |      1e-5      |    no    |            IK position solver precision           |              |
|       | maxIterPos |       int      |       |      1000      |    no    |         IK position solver max iterations         |              |
|       | vTranslMax |     double     |  m/s  |      1.0       |    no    |             maximum translation speed             |              |
|       |   vRotMax  |     double     | deg/s |      50.0      |    no    |               maximum rotation speed              |              |
|       |   lambda   |     double     |       |      0.01      |    no    |            lambda parameter for diff IK           |              |
|       | weightsJS  | vector<double> |       |                |    no    |                joint space weights                |              |
|       | weightsTS  | vector<double> |       |                |    no    |                task space weights                 |              |
|       |    mins    | vector<double> |  deg  |                |    yes   |         lower bound joint position limits         |              |
|       |    maxs    | vector<double> |  deg  |                |    yes   |         upper bound joint position limits         |              |
|       |   maxvels  | vector<double> | deg/s |                |    yes   |               joint velocity limits               |              |
