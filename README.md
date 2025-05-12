# RT2 Assignment 2 (ROS + Jupyter)
This repository provides a Jupyter-based interface to control a simulated robot in Gazebo (from the **assignment_2_2024** package). Instead of standalone Python nodes, we now use a notebook to send and cancel navigation goals, monitor robot state, and visualize both odometry and goal outcomes in real time.

## Running the Gazebo simulation
Ensure you have **ROS (Noetic or compatible)** and that the package **assignment_2_2024** is in your workspace.
Also, make sure that you have a working Python environment with **jupyter**, **ipywidgets**, **matplotlib**, and **rospy** installed.
Then, clone this repository into your ROS workspace's `src` folder and make sure you're in the `notebooks` branch:
```
cd ~/<your_workspace>/src
git clone https://github.com/FilippoGorini/assignment2_rt_ros.git
cd ~/<your_workspace>/src/assignment2_rt_ros
git checkout notebooks
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

### Running the Jupyter Interface

To open the notebook:

1. Start the Jupyter server with the following command:  
   ```
   cd ~/<your_workspace>/src/assignment2_rt_ros/notebooks
   jupyter notebook --allow-root --ip 0.0.0.0
   ```

2. From your browser on the host machine, navigate to the forwarded port (http://localhost:8888) and open `jupyter_go_to_point_ac.ipynb`.

3. Run each cell in order. You will see:
   - A map of the robot’s path (live-updating)
   - A bar chart of reached vs. not-reached goals
   - Controls to set/cancel goals
   - Live readings of position, velocity, and obstacle distance

### Notebook Features

**Goal Sending & Cancellation**  
Click **Send Goal** to send a new target; click **Cancel Goal** to abort the current one.

**Live ROS State**  
Displays the robot’s (x, y) position, linear/angular velocity, and nearest obstacle distance at 5 Hz.

**Trajectory Plot**  
Shows a blue trail of past positions, a red dot for current robot pose, and a green circle for the active goal.

**Goal Outcome Bar Chart**  
Counts how many goals succeeded vs. were canceled or preempted (preemptions are not counted as failures).

**ROS Integration**  
Publishes your targets on `/last_target` (for compatibility) and uses an ActionClient on `/reaching_goal`.




