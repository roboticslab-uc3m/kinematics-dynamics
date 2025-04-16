| Group |  Parameter   |  Type  | Units |  Default Value   | Required |                 Description                  |                                 Notes                                  |
|:-----:|:------------:|:------:|:-----:|:----------------:|:--------:|:--------------------------------------------:|:----------------------------------------------------------------------:|
|       |     name     | string |       | /CartesianServer |    no    |              local port prefix               |                                                                        |
|       |   fkPeriod   |  int   |  ms   |        20        |    no    |               FK stream period               |                                                                        |
|       |   coordRepr  | string |       |     cartesian    |    no    | coordinate representation for transform port |                cartesian, cylindrical, spherical, none                 |
|       |   angleRepr  | string |       | axisAngleScaled  |    no    |   angle representation for transform port    | axisAngle, axisAngleScaled, RPY, eulerYZ, eulerZYZ, polarAzimuth, none |
|       | angularUnits | string |       |      degrees     |    no    |   angle representation for transform port    |                            degrees, radians                            |
