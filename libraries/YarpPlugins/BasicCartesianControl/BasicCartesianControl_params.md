| Group |     Parameter      |  Type  | Units |    Default Value    | Required |                 Description                 |   Notes   |
|:-----:|:------------------:|:------:|:-----:|:-------------------:|:--------:|:-------------------------------------------:|:---------:|
|       |   controllerGain   | double |       |        0.05         |    no    |               controller gain               |           |
|       | trajectoryDuration | double |   s   |        10.0         |    no    |             trajectory duration             |           |
|       |    cmcPeriodMs     |  int   |   ms  |         50          |    no    |                  CMC rate                   |           |
|       |    waitPeriodMs    |  int   |   ms  |         30          |    no    |             wait command period             |           |
|       |    usePosdMovl     |  bool  |       |        false        |    no    | execute MOVL commands in POSD mode using IK |           |
|       |   enableFailFast   |  bool  |       |        false        |    no    |   enable fail-fast mode for MOVL commands   |           |
|       |   referenceFrame   | string |       |        base         |    no    |               reference frame               | base, tcp |
|       |       robot        | string |       | remote_controlboard |    no    |                robot device                 |           |
|       |       solver       | string |       |      KdlSolver      |    no    |           cartesian solver device           |           |
