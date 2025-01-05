#! /usr/bin/env python

import rospy
import actionlib
from nav_msgs.msg import Odometry
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from assignment2_rt_ros.msg import RobotState                       # Import the custom state message
import sys
import select


# This is the callback function called when new data is available on the subscribed /odom topic. It extracts position ...
# ... and velocity data from the odometry message and publishes it as a new RobotState message on the /robot_state topic
def odom_callback(msg):
    robot_state = RobotState()                                      # Define an empty RobotState message
    robot_state.x = msg.pose.pose.position.x                        # Get position and velocities from the Odometry message
    robot_state.y = msg.pose.pose.position.y
    robot_state.vel_x = msg.twist.twist.linear.x
    robot_state.vel_z = msg.twist.twist.angular.z
    pub_state.publish(robot_state)                                  # Publish every time new data is received on /odom topic and this callback is triggered


# This function implements a loop that prompts the user for a target point, while allowing it to cancel it at any time ...
# ... by typing "c" and "Enter". 
def send_goal():
    while not rospy.is_shutdown():
        while True:      
            try:                                                    # Try converting the input to float to be sure the user entered a number
                x = float(input("\nEnter target x: "))  
                y = float(input("Enter target y: "))
                break                                               # Break if otherwise he entered a string that can't be converted to float
            except ValueError: 
                print("\nINVALID INPUT!: Please enter numeric values for x and y")

        goal = PlanningGoal()                                       # Define an empty action goal message
        goal.target_pose.pose.position.x = x                        # Set goal x
        goal.target_pose.pose.position.y = y                        # Set goal y

        client.send_goal(goal)                                      # Send the goal to the action server
        print("\nGoal sent! Press 'c' and Enter to cancel the goal")

        # In the following loop, we didn't use the usual input() function because it would have blocked the terminal ...
        # ... waiting for user input, not checking if the goal was reached in the meanwhile. This code instead uses ...
        # ... the select.select system call to check if the user has typed something but doesn't wait (timeout is set to 0)
        while client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.PREEMPTED]:  
            if select.select([sys.stdin], [], [], 0)[0]:            # Check if there is user input available WITHOUT blocking execution  
                user_input = sys.stdin.readline().strip()           # Read the input (non-blocking because of select)  
                if user_input.lower() == 'c':                       # If the user types 'c', cancel the goal  
                    client.cancel_goal()
                    print("\nGoal was canceled!")  
                    break                                           # Exit the loop since the goal has been canceled  

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print("\nGoal reached successfully!")


# This is the main function of the node, that initializes a client for the /reaching_goal action server, plus a subscriber ...
# ... and a publisher to monitor the state of the robot
def main():

    global sub_odom, pub_state, client
    rospy.init_node("go_to_point_client")

    client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)     # Setup the action client
    print("\nWaiting for action server...")
    client.wait_for_server()
    print("Connected to action server!")

    sub_odom = rospy.Subscriber("/odom", Odometry, odom_callback)               # Setup the subscriber to the /odom topic
    pub_state = rospy.Publisher("/robot_state", RobotState, queue_size=10)      # Setup publisher for the robot state

    while not rospy.is_shutdown():                                              # Because of the loop, no spin function is required (at least here in python)
        send_goal()


if __name__ == "__main__":
    main()
