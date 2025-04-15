| Group |    Parameter   |      Type      | Units |          Default Value          | Required |                    Description                    | Notes |
|:-----:|:--------------:|:--------------:|:-----:|:-------------------------------:|:--------:|:-------------------------------------------------:|:-----:|
|       |      quiet     |      bool      |       |              false              |    no    |                  disable logging                  |       |
|       |   kinematics   |     string     |       |                                 |    no    | path to file with description of robot kinematics |       |
|       |     gravity    | vector<double> | m/s^2 |           (0.0 0.0 9.81)        |    no    |                   gravity vector                  |       |
|       |      ikPos     |     string     |       |                st               |    no    |  IK position solver algorithm (lma, nrjl, st, id) |       |
|       |      ikVel     |     string     |       |               pinv              |    no    |     IK velocity solver algorithm (pinv, wdls)     |       |
|       |     epsPos     |     double     |   m   |               1e-5              |    no    |            IK position solver precision           |       |
|       |   maxIterPos   |       int      |       |               1000              |    no    |         IK position solver max iterations         |       |
|       |     epsVel     |     double     |   m   |               1e-5              |    no    |            IK velocity solver precision           |       |
|       |   maxIterVel   |       int      |       |               150               |    no    |         IK velocity solver max iterations         |       |
|       |     lambda     |     double     |       |               0.01              |    no    |            lambda parameter for diff IK           |       |
|       |   weightsLMA   | vector<double> |       |    (1.0 1.0 1.0 0.1 0.1 0.1)    |    no    |               LMA algorithm weights               |       |
|       |   weightsJS    | vector<double> |       |               (0.0)             |    no    |                joint space weights                |       |
|       |   weightsTS    | vector<double> |       |               (0.0)             |    no    |                task space weights                 |       |
|       | invKinStrategy |     string     |       | leastOverallAngularDisplacement |    no    |             IK configuration strategy             |       |
|       |      mins      | vector<double> |  deg  |               (0.0)             |    no    |         lower bound joint position limits         |       |
|       |      maxs      | vector<double> |  deg  |               (0.0)             |    no    |         upper bound joint position limits         |       |
