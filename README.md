IMU tools for ROS
===================================

Overview
-----------------------------------

IMU-related filters and visualizers. The stack contains:

 * `csm`: a meta-package that downloads and installs Andrea Censi's 
Canonical Scan Matcher [1] locally

 * `laser_ortho_projector`: calculates orthogonal projections of LaserScan messages
 
 * `laser_scan_matcher`: an incremental laser scan matcher, using Andrea Censi's Canonical 
Scan Matcher implementation

 * `laser_scan_sparsifier`: takes in a LaserScan message and sparsifies it

 * `laser_scan_splitter`:  takes in a LaserScan message and splits 
it into a number of other LaserScan messages 

 * `ncd_parser`: reads in .alog data files from the New College Dataset [2]
and broadcasts scan and odometry messages to ROS.

 * `scan_to_cloud_converter`: converts LaserScan to PointCloud messages.

Installing
-----------------------------------

### From source ###

Create a directory where you want the package downloaded (ex. `~/ros`), 
and add it to `$ROS_PACKAGE_PATH`.

Make sure you have git installed:

    sudo apt-get install git-core

Download the stack from our repository:

    git clone https://github.com/ccny-ros-pkg/scan_tools.git

Install any dependencies using [[rosdep]].

    rosdep install scan_tools

Compile the stack:

    rosmake scan_tools

More info
-----------------------------------

http://ros.org/wiki/scan_tools

References
-----------------------------------
 [1] A. Censi, "An ICP variant using a point-to-line metric" Proceedings of the 
IEEE International Conference on Robotics and Automation (ICRA), 2008

 [2] M. Smith, I. Baldwin, W. Churchill, R. Paul, and P. Newman, 
The new college vision and laser data set, International Journal for Robotics 
Research (IJRR), vol. 28, no. 5, pp. 595599, May 2009.
 
