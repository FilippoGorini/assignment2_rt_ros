.. _usage:

Usage
=====

Launching the System
--------------------

To start the Gazebo simulation environment and run the nodes provided by this package, execute the main launch file:

.. code-block:: bash

   roslaunch assignment2_rt_ros assignment2.launch

This launch file handles starting Gazebo, spawning the robot model, and running the necessary nodes from both this package and its dependencies (like ``assignment_2_2024``).

Monitoring Robot State
----------------------

You can monitor the robot's published state (position and velocity) in a separate terminal:

.. code-block:: bash

   rostopic echo /robot_state