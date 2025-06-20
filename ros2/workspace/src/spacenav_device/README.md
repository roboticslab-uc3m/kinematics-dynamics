# Spacenav Device Node

The **spacenav_device** node allows the control of a generic robot suscribing to Joy Msgs in ROS 2 and publishing data according to different control modes. This was initially created to use a SpaceMouse device by subscribing to the its data and the current robot positions, while publishing the relevant information to another node, such as a [Cartesian Control Server](https://github.com/mercerebo1/cartesian-controllers), for teleoperation of the robot. Depending on the configured parameters, the device's movements can be interpreted as **Twist** (velocity), **Pose** (position), or **Wrench** (force-torque) commands. Additionally, it can control actions on a TCP actuator, such as opening and closing a gripper, based on the state of the SpaceMouse buttons.

Original work: María de las Mercedes Rebollo Rayo, *Implementación en ROS 2 de un controlador genérico para teleoperación de un robot manipulador mediante un ratón 3D*, Trabajo Fin de Grado, Universidad Carlos III de Madrid, 2024. https://github.com/mercerebo1/spacenav_device

## Program Description

The structure and functionality of the program are detailed below:

### 1. Required Libraries

- **spacenav_device.hpp**: Contains the declaration of the `SpacenavSubscriber` class.
- **ICartesianControl.h**: Interface for Cartesian control provided by the RoboticsLab `kinematics-dynamics` library.
- **geometry_msgs**: Standard ROS messages for points, vectors, and positions.
- **std_msgs**: Standard ROS messages for sharing basic data.
- **rclcpp**: Provides essential C++ API for interacting with ROS.
- **tf2**: Tools for transforming points, vectors, and positions for kinematic calculations.
- Other standard C++ libraries like `vector`, `string`, and `mutex`.

### 2. Constants

- `DEFAULT_NODE_NAME`: Default name for the node.
- `DEFAULT_AXIS_SCALE`: Default scale for axes.
- `DEFAULT_STREAMING_MSG`: Default type of streaming message.

### 3. SpacenavSubscriber Class

#### Constructor:

- Initializes the last update time for peripheral data acquisition.
- Declares and retrieves node parameters (`streaming_msg` and `scale`).
- Adds a callback for parameter changes.
- Creates a client for the `set_parameters` service and waits for service availability.
- Subscribes to different topics and creates a timer for periodic publications.
- Creates several publishers for different message types (Twist, Pose, Wrench, Int32).
- Validates parameters and sets default streaming commands.

#### Destructor:

- Default destructor.

#### Callbacks:

- **spnav_callback**: Handles SpaceMouse data and publishes messages based on button states and axis positions.
- **state_callback**: Retrieves the robot's initial position and orientation.
- **parameter_callback**: Handles node parameter changes.
- **timer_callback**: Publishes messages at 50 Hz depending on the configured streaming message type (Twist, Pose, or Wrench).

#### Auxiliary Methods:

- **set_preset_streaming_cmd**: Configures the streaming command on an external node.

## Key Elements

### 1. Node Parameters

- `streaming_msg`: Can be Twist, Pose, or Wrench.
- `scale`: Scale factor for SpaceMouse readings.

### 2. Error Handling

- Validation of parameters and exception handling to avoid runtime errors.

### 3. Publishers and Subscribers

- Subscribes to **Joy** messages from the SpaceMouse and **PoseStamped** messages from the robot's state.
- Publishes **Twist**, **Pose**, **Wrench**, and **Int32** messages based on the current state and configuration.

### 4. Timer

- Publishes messages at a rate of 50 Hz to ensure constant command updates from the SpaceMouse. A timer was introduced to avoid delays and saturation of messages at higher frequencies.

### 5. Mutex Usage

- Uses `std::mutex` and `std::lock_guard` to ensure data consistency between the asynchronous SpaceMouse callback and the timer.

### 6. Callbacks

- Callbacks handle input data from the SpaceMouse and the robot's current position, as well as changes in node parameters. These are triggered when messages are received from topics like `/state/pose` or `/spacenav/joy` or when parameter values are changed.

### 7. Virtual Point in Pose Messages

- In `spnav_callback`, a virtual point is introduced to describe the robot’s initial position to avoid issues with PID controller compensation in simulators like Gazebo. Subsequent trajectories are calculated from this point to ensure accurate real-time feedback from the robot.

## Contributing
Feel free to submit issues or pull requests to improve the project. Contributions are welcome!
