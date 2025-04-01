#! /usr/bin/env python

"""
.. module:: go_to_point_ac
    :platform: Unix
    :synopsis: ROS action client node for sending navigation goals.

.. moduleauthor:: Gorini Filippo - s4828475@studenti.unige.it

Overview
--------
This node implements a ROS action client allowing a user to specify (x, y) target
coordinates for robot navigation. It interacts with an action server (expected on
`/reaching_goal`) to handle the navigation task. The node also subscribes to
`/odom` to monitor the robot's state, publishes a simplified `RobotState` message,
and allows the user to cancel ongoing goals via keyboard input.

ROS Interfaces
--------------
*   **Subscribes to**:
        `/odom` (`nav_msgs.msg.Odometry`): To receive robot pose and velocity updates.

*   **Publishes to**:
        `/robot_state` (`assignment2_rt_ros.msg.RobotState`): Publishes current robot position (x, y) and velocities (linear x, angular z).
        
        `/last_target` (`assignment2_rt_ros.msg.Target`): Publishes the most recently entered target coordinates.

*   **Action Client**:
        `/reaching_goal` (`assignment_2_2024.msg.PlanningAction`): Sends `PlanningGoal` messages containing target coordinates to the navigation action server.

Functions
---------

"""

# import rospy
# import actionlib
# from nav_msgs.msg import Odometry
# from assignment_2_2024.msg import PlanningAction, PlanningGoal
# from assignment2_rt_ros.msg import RobotState, Target
# import sys
# import select


def odom_callback(msg):
    """
    Callback function executed when a message is received on the `/odom` topic.

    It extracts the robot's planar position (x, y), linear velocity (vel_x),
    and angular velocity (ang_vel_z) from the received `Odometry` message.
    This information is then published as a `RobotState` message onto the
    `/robot_state` topic.

    Args:
        msg (nav_msgs.msg.Odometry): The incoming odometry data.
    """
    robot_state = RobotState()                                      # Define an empty RobotState message
    robot_state.x = msg.pose.pose.position.x                        # Get position and velocities from the Odometry message
    robot_state.y = msg.pose.pose.position.y
    robot_state.vel_x = msg.twist.twist.linear.x
    robot_state.ang_vel_z = msg.twist.twist.angular.z               # Angular velocity, not linear!
    pub_state.publish(robot_state)                                  # Publish every time new data is received on /odom topic and this callback is triggered


def send_goal():
    """
    Handles the user interaction for setting and managing navigation goals.

    Prompts the user for target (x, y) coordinates, performing basic input validation.
    Publishes the entered target to `/last_target`. Constructs and sends a `PlanningGoal`
    to the `/reaching_goal` action server. While the goal is active, it monitors
    for user keyboard input ('c' + Enter) using `select.select` for non-blocking
    cancellation. Reports the final goal status (Success, Cancelled, Aborted).
    """
    while not rospy.is_shutdown():
        while True:      
            try:                                                    # Try converting the input to float to be sure the user entered a number
                x = float(input("\nEnter target x: "))  
                y = float(input("Enter target y: "))
                break                                               # Break if otherwise he entered a string that can't be converted to float
            except ValueError: 
                print("\nINVALID INPUT!: Please enter numeric values for x and y")

        pub_target.publish(Target(x, y))                            # Publish last target point set by the user using custom target message

        goal = PlanningGoal()                                       # Define an empty action goal message
        goal.target_pose.pose.position.x = x                        # Set goal x
        goal.target_pose.pose.position.y = y                        # Set goal y

        client.send_goal(goal)                                      # Send the goal to the action server
        print("\nGoal sent! Press 'c' and then 'Enter' to cancel the goal")

        # In the following loop, we didn't use the usual input() function because it would have blocked the terminal ...
        # ... waiting for user input, not checking if the goal was reached in the meanwhile. This code instead uses ...
        # ... the select.select system call to check if the user has typed something but doesn't wait (timeout is set to 0)
        while client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:  
            if select.select([sys.stdin], [], [], 0)[0]:            # Check if there is user input available WITHOUT blocking execution  
                user_input = sys.stdin.readline().strip()           # Read the input (non-blocking because of select)  
                if user_input.lower() == "c":                       # If the user types 'c', cancel the goal  
                    client.cancel_goal()
                    print("\nGoal was canceled!")  
                    break                                           # Exit the loop since the goal has been canceled  

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print("\nGoal reached successfully!")


def main():
    """
    Main function: Initializes the ROS node and manages overall execution.

    - Initializes the ROS node named "go_to_point_ac_node".
    - Creates the action client for `/reaching_goal` and waits for the server.
    - Sets up the subscriber to `/odom` with `odom_callback`.
    - Sets up the publishers for `/robot_state` and `/last_target`.
    - Enters the main loop, repeatedly calling `send_goal()` to handle goal cycles.
    """
    global sub_odom, pub_state, pub_target, client
    rospy.init_node("go_to_point_ac_node")

    client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)     # Setup the action client
    print("\nWaiting for action server...")
    client.wait_for_server()
    print("Connected to action server!")

    sub_odom = rospy.Subscriber("/odom", Odometry, odom_callback)               # Setup the subscriber to the /odom topic
    pub_state = rospy.Publisher("/robot_state", RobotState, queue_size=10)      # Setup publisher for the robot state
    pub_target = rospy.Publisher("/last_target", Target, queue_size=10)         # Setup publisher for the last target

    while not rospy.is_shutdown():                                              # Because of the loop, no spin function is required (at least here in python)
        send_goal()


if __name__ == "__main__":
    main()