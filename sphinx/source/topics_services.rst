.. _topics_services:

Topics & Services
=================

This package interacts with ROS through the following topics, services, and actions.

Published Topics
----------------

* ``/robot_state`` (``assignment2_rt_ros/RobotState``)
   Publishes the robot's current position (x, y), linear velocity x velocity and z angular velocity. Uses a custom message type.
* ``/last_target`` (``assignment2_rt_ros/Target``)
   Stores and publishes the coordinates (x, y) of the last goal received by the action client. Uses a custom message type.

Subscribed Topics
-----------------

* ``/odom`` (``nav_msgs/Odometry``)
   Subscribes to the robot's odometry information to determine its current position and velocity.


Advertised Services
-------------------

* **get_last_target** (``assignment2_rt_ros/GetLastTarget``)
   Provides a service server that, when called, returns the last goal position (x, y) that was sent by the user/action client.


Action Clients
--------------

* **/reaching_goal** (Action Type defined in ``assignment_2_2024``)
   This package implements an action *client* that sends navigation goals (target x, y coordinates) to an action *server* running on this topic. The server is expected to be provided by the ``assignment_2_2024`` package.