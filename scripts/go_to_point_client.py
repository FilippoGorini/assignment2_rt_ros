#! /usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from assignment2_rt_ros.msg import RobotState                       # Import the custom state message


def odom_callback(msg):
    """Callback function for odometry subscriber to extract position and velocity."""
    robot_state = RobotState()                                      # Define an empty RobotState message
    robot_state.x = msg.pose.pose.position.x                        # Get position and velocities from the Odometry message
    robot_state.y = msg.pose.pose.position.y
    robot_state.vel_x = msg.twist.twist.linear.x
    robot_state.vel_z = msg.twist.twist.angular.z
    pub_state.publish(robot_state)                                  # Publish every time new data is received on /odom topic and this callback is triggered


def send_goal():
    """Prompt the user for a goal and send it to the action server."""
    while not rospy.is_shutdown():
        x = float(input("Enter target x: "))
        y = float(input("Enter target y: "))

        goal = PlanningGoal()                                       # Define an empty action goal message
        goal.target_pose.pose.position.x = x                        # Set goal x
        goal.target_pose.pose.position.y = y                        # Set goal y

        client.send_goal(goal, feedback_cb=feedback_callback)       # Send the goal to the action server using feedback_callback as callback ...
                                                                    # ... for when the server updates the feedback topic

        print("Goal sent. Type 'cancel' to stop execution or wait for completion.")

        while client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            user_input = input()
            if user_input.lower() == 'cancel':
                client.cancel_goal()
                print("Goal canceled.")
                break


def feedback_callback(feedback):
    """Callback function to display action server feedback."""
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        print("Target reached!")



if __name__ == "__main__":
    rospy.init_node("go_to_point_client")

    client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)     # Setup the action client
    print("Waiting for action server...")
    client.wait_for_server()
    print("Connected to action server!")

    pub_state = rospy.Publisher("/robot_state", RobotState, queue_size=10)      # Setup publisher for the robot state
    sub_odom = rospy.Subscriber("/odom", Odometry, odom_callback)               # Setup the subscriber to the /odom topic

    while not rospy.is_shutdown():                                      # Because of the loop, no spin function is required (at least here in python)
        send_goal()
