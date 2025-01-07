# RT1 Assignment 2 (ROS)
This repository contains a ROS package implementing two nodes: `go_to_point_client.py` and `get_last_target.py`. 
These nodes interact with the Gazebo simulation provided in the package **assignment_2_2024** to control the robot movement towards a target in space set by the user.

## Running the Nodes
Ensure you have **ROS (Noetic or compatible)** and that the package **assignment_2_2024** is in your workspace.
Then, clone this repository into your ROS workspace's `src` folder:
```
cd ~/<your_workspace>/src
git clone https://github.com/FilippoGorini/assignment2_rt_ros.git
```

Build the package:
```
cd ~/<your_workspace>
catkin_make
```

Source ROS and your workspace in the `.bashrc` file:
```
source /opt/ros/noetic/setup.bash  
source ~/<your_workspace>/devel/setup.bash
```

Once this is done, you can run the whole simulation, together with the nodes, using the following command:
```
roslaunch assignment2_rt_ros assignment2.launch
```
Notice that in this case, because we're using a launchfile, it is not necessary to run the ROS master node in advance.


## Node `go_to_point_ac.py` - Action Client for Navigation
This node implements an action client which communicates with the action server already implemented in `bug_as.py` from the **assignment_2_2024** package.
In particular, this node allows the user to set (or cancel) a goal for the robot to reach, while also publishing the state of the robot to a topic.
To listen to this topic, open a new terminal and run the following command:
```
rostopic echo /robot_state
```
Once the launchfile is executed, the node will automatically run and ask the user to enter the coordinates of the target position.

### Main Features
- Sends goal positions to the `bug_as.py` action server.
- Allows to cancel the goal before the robot actually reaches the target.
- Monitors feedback from the action server to determine when the goal has been reached.
- Publishes the robot's position (x, y) and velocity (vel_x, vel_z) on the `/robot_state` topic, using the RobotState custom message.

### Code Structure
The `go_to_point_ac.py` node code is structured into three main components:

- **`main()`**: Initializes the ROS node and sets up:
    - An action client for `/reaching_goal`.
    - A subscriber to `/odom` to store the latest data about the robot.
    - Publishers for `/robot_state` and `/last_target`.
    - Calls send_goal in an infinite loop until ROS shuts down.

- **`odom_callback()`**:
    - Extracts x, y, linear velocity (vel_x), and angular velocity (ang_vel_z).
    - Publishes the extracted information as a custom RobotState message on the /robot_state topic.

- **`send_goal()`**:
    - Continuously prompts the user for a target position (x, y).
    - Publishes the target coordinates on the `/last_target` topic.
    - Sends the goal to the `/reaching_goal` action server.
    - Uses a non-blocking input method (`select.select`) to allow goal cancellation ("c" key) while monitoring the action server's state.
    - Prints messages indicating whether the goal was successfully reached or canceled.

Additionally, checks are performed to ensure that the user input is a valid numerical value and not a string.


## Node `get_last_target_srv.py` - Service for Target Monitoring
This node implements a service server which is able to retrieve and return the last target set by the user. The node is run in the background as soon as the launchfile is executed, 
therefore to test it just call the service like this:
```
rosservice call get_last_target
```

### Main Features
- Implements a ROS service named `get_last_target` that returns the last goal position.
- Stores the last received target, using a custom `Target` message by subscribing to the `/last_target` topic.
- Responds to service calls with the last received goal.

### Code Structure
The `get_last_target_srv.py` node is quite simple, consisting of only three functions:

- **`main()`**: Initializes the node, a subscriber to the `last_target` topic and the service itself (`get_last_target`).
- **`target_callback()`**: Stores the message received from the topic in a variable to be able to retrieve it later.
- **`handle_get_last_target()`**: When the service is called this function returns the last target set by the user.





