#! /usr/bin/env python

"""
.. module:: got_to_point_ac
   :platform: Unix
   :synopsis: This module contains the code for the ROS1 package of the assignment 2.

.. moduleauthor:: Gorini Filippo - s4828475@studenti.unige.it

Test
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
    Callback function for the `/odom` topic.

    This function is triggered whenever new odometry data is received. It extracts 
    position and velocity information from the message and republishes it as a 
    `RobotState` message on the `/robot_state` topic.

    Args:
        msg (nav_msgs.msg.Odometry): The received `Odometry` message containing position and velocity.
    """
    robot_state = RobotState()
    robot_state.x = msg.pose.pose.position.x
    robot_state.y = msg.pose.pose.position.y
    robot_state.vel_x = msg.twist.twist.linear.x
    robot_state.ang_vel_z = msg.twist.twist.angular.z  # Angular velocity, not linear!
    pub_state.publish(robot_state)


def send_goal():
    """
    Handles user input to send target goals and allows goal cancellation.

    This function continuously prompts the user to enter a target `(x, y)` position.
    It then publishes the target and sends a goal to the action server. The user 
    can cancel the goal by typing `"c"` and pressing Enter.

    The function uses `select.select` to check for user input without blocking execution.
    """
    while not rospy.is_shutdown():
        while True:
            try:
                x = float(input("\nEnter target x: "))  
                y = float(input("Enter target y: "))
                break
            except ValueError: 
                print("\nINVALID INPUT!: Please enter numeric values for x and y")

        pub_target.publish(Target(x, y))  # Publish last target point set by the user

        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        client.send_goal(goal)
        print("\nGoal sent! Press 'c' and then 'Enter' to cancel the goal")

        while client.get_state() not in [actionlib.GoalStatus.SUCCEEDED, 
                                         actionlib.GoalStatus.ABORTED, 
                                         actionlib.GoalStatus.PREEMPTED]:  
            if select.select([sys.stdin], [], [], 0)[0]:  # Non-blocking check for user input  
                user_input = sys.stdin.readline().strip()  
                if user_input.lower() == "c":  # Cancel the goal if user enters 'c'
                    client.cancel_goal()
                    print("\nGoal was canceled!")  
                    break  

        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            print("\nGoal reached successfully!")


def main():
    """
    Main function that initializes the ROS node and sets up communication.

    This function:
    - Initializes a ROS node.
    - Creates an action client for the `/reaching_goal` action.
    - Subscribes to the `/odom` topic to receive robot state updates.
    - Publishes robot state updates to `/robot_state`.
    - Publishes the last user-defined target to `/last_target`.

    The function then enters a loop to continuously accept user input and send goals.
    """
    global sub_odom, pub_state, pub_target, client
    rospy.init_node("go_to_point_ac_node")

    client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)
    print("\nWaiting for action server...")
    client.wait_for_server()
    print("Connected to action server!")

    sub_odom = rospy.Subscriber("/odom", Odometry, odom_callback)
    pub_state = rospy.Publisher("/robot_state", RobotState, queue_size=10)
    pub_target = rospy.Publisher("/last_target", Target, queue_size=10)

    while not rospy.is_shutdown():
        send_goal()


if __name__ == "__main__":
    main()
