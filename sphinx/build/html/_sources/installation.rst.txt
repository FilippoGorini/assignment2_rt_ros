.. _installation:

Installation
============

Prerequisites
-------------

* **ROS Noetic** (or a compatible distribution) must be installed.
* The dependency package **assignment_2_2024** must be present and built in your Catkin workspace. This package provides the action server that ``assignment2_rt_ros`` communicates with.

Steps
-----

1.  Navigate to your Catkin workspace's ``src`` directory:

   .. code-block:: bash

      cd ~/<your_workspace>/src

2.  Clone this repository:

   .. code-block:: bash

      git clone https://github.com/FilippoGorini/assignment2_rt_ros.git

3.  Build your workspace:

   .. code-block:: bash

      cd ~/<your_workspace>
      catkin_make

4.  Source your ROS environment and workspace setup files:

   .. code-block:: bash

      # Source the main ROS setup file
      source /opt/ros/noetic/setup.bash
      # Source your workspace's setup file
      source ~/<your_workspace>/devel/setup.bash