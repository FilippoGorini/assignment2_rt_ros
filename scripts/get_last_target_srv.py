#! /usr/bin/env python

import rospy
from assignment2_rt_ros.msg import Target
from assignment2_rt_ros.srv import GetLastTarget, GetLastTargetResponse


# This callback function updates the last_target global variable every time new data is received on the /last_target topic
def target_callback(msg):
    global last_target
    last_target = msg                              

# This function returns the response the server should give back to the client who sent the request
def handle_get_last_target(req):
    return GetLastTargetResponse(last_target)

# This is the main function, which simply initializes a subscriber to the /last_target topic and implements the service server ...
# ... which returns the last target sent when called
def main():

    global last_target 
    last_target = Target()
    rospy.init_node('get_last_target_srv_node')

    rospy.Subscriber('/last_target', Target, target_callback)
    rospy.Service('get_last_target', GetLastTarget, handle_get_last_target)

    rospy.spin()                   


if __name__ == "__main__":
    main()
