#! /usr/bin/env python

"""
.. module:: get_last_target_srv
    :platform: Unix
    :synopsis: ROS service node for retrieving the last target position.

.. moduleauthor:: Gorini Filippo - s4828475@studenti.unige.it

Overview
--------
This node implements a ROS service that allows other nodes to retrieve the last target
position (x, y coordinates) set by the user or another node. The last target position
is stored internally and updated whenever a new message is received on the `/last_target`
topic. The service `/get_last_target` provides access to this stored target.

ROS Interfaces
--------------
*   **Subscribes to**:
        `/last_target` (`assignment2_rt_ros.msg.Target`): Receives the latest target coordinates to be stored.

*   **Service Server**:
        `get_last_target` (`assignment2_rt_ros.srv.GetLastTarget`): Responds to requests with the last received target coordinates.

Functions
---------

"""


# import rospy
# from assignment2_rt_ros.msg import Target
# from assignment2_rt_ros.srv import GetLastTarget, GetLastTargetResponse

def target_callback(msg):
    """
    Callback function executed when a message is received on the `/last_target` topic.

    Updates the globally stored `last_target` variable with the position data
    from the incoming message.

    Args:
        msg (assignment2_rt_ros.msg.Target): The received target message containing the latest x and y coordinates.
    """
    global last_target
    last_target = msg

def handle_get_last_target(req):
    """
    Service callback function for handling requests to the `get_last_target` service.

    This function is called when a client requests the last target. It retrieves the
    globally stored `last_target` and includes it in the service response.

    Args:
        req (assignment2_rt_ros.srv.GetLastTargetRequest): The incoming service request object (content is not used in this case).

    Returns:
        assignment2_rt_ros.srv.GetLastTargetResponse: The service response containing the last stored target coordinates.
    """
    return GetLastTargetResponse(last_target)

def main():
    """
    Main function: Initializes the ROS node and manages overall execution.

    - Initializes the ROS node named `get_last_target_srv_node`.
    - Initializes the global `last_target` variable with default values.
    - Sets up the subscriber to the `/last_target` topic, using `target_callback` to handle incoming messages.
    - Sets up the service server for `get_last_target`, using `handle_get_last_target` to process requests.
    - Enters the ROS spin loop (`rospy.spin()`) to keep the node running and responsive to callbacks and service requests.
    """
    global last_target 
    last_target = Target()
    rospy.init_node('get_last_target_srv_node')

    rospy.Subscriber('/last_target', Target, target_callback)
    rospy.Service('get_last_target', GetLastTarget, handle_get_last_target)

    rospy.spin()    

if __name__ == "__main__":
    main()