#! /usr/bin/env python

import rospy
from assignment2_rt_ros.msg import Target
from assignment2_rt_ros.srv import GetLastTarget, GetLastTargetResponse


def target_callback(msg):
    """
    Callback function for the `/last_target` topic.

    This function is triggered every time new data is received on the `/last_target` topic.
    It updates the global `last_target` variable with the new `Target` message.

    Args:
        msg (assignment2_rt_ros.msg.Target): The received `Target` message containing the target coordinates.
    """
    global last_target
    last_target = msg


def handle_get_last_target(req):
    """
    Handles a service request to get the last target.

    This function returns the `last_target` message as the response to the client that called the service.

    Args:
        req (assignment2_rt_ros.srv.GetLastTargetRequest): The request message (not used in this case).

    Returns:
        GetLastTargetResponse: The response message containing the last target data.
    """
    return GetLastTargetResponse(last_target)


def main():
    """
    Main function that initializes the ROS node and sets up the subscriber and service.

    This function:
    - Initializes the ROS node `get_last_target_srv_node`.
    - Creates a subscriber to the `/last_target` topic to receive updates.
    - Creates a service `get_last_target` that allows clients to request the last target.

    The function then enters a loop with `rospy.spin()` to process incoming requests.
    """
    global last_target 
    last_target = Target()
    rospy.init_node('get_last_target_srv_node')

    rospy.Subscriber('/last_target', Target, target_callback)
    rospy.Service('get_last_target', GetLastTarget, handle_get_last_target)

    rospy.spin()


if __name__ == "__main__":
    main()
