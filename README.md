imu_tools stack for ROS
===================================

Overview
-----------------------------------
This stack contains IMU-related filters and visualizers.

Installing
-----------------------------------

### From source ###

Create a directory where you want the package downloaded (ex. `~/ros`), 
and add it to $ROS_PACKAGE_PATH.

Make sure you have git installed:

    sudo apt-get install git-core

Download the stack from our repository:

    git clone https://github.com/ccny-ros-pkg/imu_tools.git

Install any dependencies using [[rosdep]].

    rosdep install imu_tools

Compile the stack:

    rosmake imu_tools

