| Group | Parameter | Type   | Units | Default Value | Required | Description                                            | Notes                      |
|:-----:|:---------:|:------:|:-----:|:--------------|:--------:|:------------------------------------------------------:|:--------------------------:|
|       | planeA    | double | m     | 0.0           | yes      | First coefficient for the plane equation               |                            |
|       | planeB    | double | m     | 0.0           | yes      | Second coefficient for the plane equation              |                            |
|       | planeC    | double | m     | 0.0           | yes      | Third coefficient for the plane equation               |                            |
|       | planeD    | double | m     | 0.0           | yes      | Fourth coefficient for the plane equation              |                            |
|       | threshold | double | m     | 0.1           | yes      | Distance from the plane at which to trigger the sensor | y = threshold + factor / x |
|       | factor    | double | -     | 1.0           | yes      | Scaling factor for the sensor reading                  | y = threshold + factor / x |
