#! /usr/bin/env python

"""
.. module:: get_last_target_srv
   :platform: Unix
   :synopsis: ROS service node for retrieving the last target position.

.. moduleauthor:: Gorini Filippo - s4828475@studenti.unige.it

This node implements a ROS service that allows other nodes to retrieve the last target position
set by the user. The last target is updated through a subscriber to the `/last_target` topic.

**ROS Topics:**
    - **Subscribes to**:
        - `/last_target` (`assignment2_rt_ros.msg.Target`): Receives the latest target position.

**ROS Service:**
    - **Service Server**:
        - `get_last_target` (`assignment2_rt_ros.srv.GetLastTarget`): Returns the last received target.
"""


import rospy
from assignment2_rt_ros.msg import Target
from assignment2_rt_ros.srv import GetLastTarget, GetLastTargetResponse

def target_callback(msg):
    """
    Callback function for the `/last_target` topic.

    Updates the global `last_target` variable whenever a new `Target` message is received.

    Args:
        msg (assignment2_rt_ros.msg.Target): The received target message containing position data.
    """
    global last_target
    last_target = msg

def handle_get_last_target(req):
    """
    Service callback for the `get_last_target` service.

    Returns the last received target position.

    Args:
        req (assignment2_rt_ros.srv.GetLastTargetRequest): The service request (not used).

    Returns:
        GetLastTargetResponse: The response containing the last target position.
    """
    return GetLastTargetResponse(last_target)

def main():
    """
    Initializes the ROS node and sets up the subscriber and service.

    - **Node Name**: `get_last_target_srv_node`
    - **Subscribes to**: `/last_target` (`assignment2_rt_ros/Target`)
    - **Provides Service**: `get_last_target` (`assignment2_rt_ros/GetLastTarget`)

    The node continuously listens for new target positions and serves the last recorded target 
    through a ROS service.
    """
    global last_target
    last_target = Target()

    rospy.init_node('get_last_target_srv_node')
    rospy.Subscriber('/last_target', Target, target_callback)
    rospy.Service('get_last_target', GetLastTarget, handle_get_last_target)

    rospy.spin()

if __name__ == "__main__":
    main()
