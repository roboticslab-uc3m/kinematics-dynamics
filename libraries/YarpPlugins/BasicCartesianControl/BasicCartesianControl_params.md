| Group |     Parameter      |  Type  | Units |    Default Value    | Required |                 Description                 |                  Notes                   |
|:-----:|:------------------:|:------:|:-----:|:-------------------:|:--------:|:-------------------------------------------:|:----------------------------------------:|
|       |   controllerGain   | double |       |        0.05         |    no    |               controller gain               |                                          |
|       | trajectoryDuration | double |   s   |         0.0         |    no    |             trajectory duration             | 0: use ref speed/acc to compute duration |
|       | trajectoryRefSpeed | double |  m/s  |        0.05         |    no    |      trajectory reference linear speed      |                                          |
|       | trajectoryRefAccel | double | m/s^2 |        0.02         |    no    |  trajectory reference linear acceleration   |                                          |
|       |    cmcPeriodMs     |  int   |   ms  |         50          |    no    |                  CMC rate                   |                                          |
|       |    waitPeriodMs    |  int   |   ms  |         30          |    no    |             wait command period             |                                          |
|       |    usePosdMovl     |  bool  |       |        false        |    no    | execute MOVL commands in POSD mode using IK |                                          |
|       |   enableFailFast   |  bool  |       |        false        |    no    |   enable fail-fast mode for MOVL commands   |                                          |
|       |   referenceFrame   | string |       |        base         |    no    |               reference frame               |                base, tcp                 |
|       |       robot        | string |       | remote_controlboard |    no    |                robot device                 |                                          |
|       |       solver       | string |       |      KdlSolver      |    no    |           cartesian solver device           |                                          |
