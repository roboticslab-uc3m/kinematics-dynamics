| Group |    Parameter   |      Type      | Units |          Default Value          | Required |            Description            | Notes |
|:-----:|:--------------:|:--------------:|:-----:|:-------------------------------:|:--------:|:---------------------------------:|:-----:|
|       |       A0       |     double     |   m   |               0.3               |    no    |          length of link 1         |       |
|       |       A1       |     double     |   m   |               0.4               |    no    |          length of link 2         |       |
|       |       A2       |     double     |   m   |               0.4               |    no    |          length of link 3         |       |
|       |       A3       |     double     |   m   |               0.3               |    no    |          length of link 4         |       |
|       | invKinStrategy |     string     |       | leastOverallAngularDisplacement |    no    |     IK configuration strategy     |       |
|       |      mins      | vector<double> |  deg  |                                 |    yes   | lower bound joint position limits |       |
|       |      maxs      | vector<double> |  deg  |                                 |    yes   | upper bound joint position limits |       |
