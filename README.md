IMU tools for ROS
=================

Overview
--------

IMU-related filters and visualizers. The repository contains:

* `imu_filter_madgwick`: a filter which fuses angular velocities,
  accelerations, and (optionally) magnetic readings from a generic IMU
  device into an orientation. Based on the work of [1].

* `imu_complementary_filter`: a filter which fuses angular velocities,
  accelerations, and (optionally) magnetic readings from a generic IMU
  device into an orientation quaternion using a novel approach based on a
  complementary fusion. Based on the work of [2].

* `rviz_imu_plugin` a plugin for rviz which displays `sensor_msgs::Imu` messages

[1]: https://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

[2]: https://www.mdpi.com/1424-8220/15/8/19302


Installing
----------

### From binaries

This repo has been released into all current ROS1 and ROS2 distros. To install,
simply:

    sudo apt-get install ros-<YOUR_ROSDISTO>-imu-tools

### From source (ROS1)

[Create a catkin workspace](https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
(e.g., `~/catkin_ws/`) and source the `devel/setup.bash` file.

Make sure you have git installed:

    sudo apt-get install git

Clone this repository into your catkin workspace (e.g., `~/catin_ws/src`; use
the proper branch for your distro, e.g., `melodic`, `noetic`, ...):

    git clone -b <YOUR_ROSDISTO> https://github.com/CCNYRoboticsLab/imu_tools.git

Install any dependencies using [rosdep](https://www.ros.org/wiki/rosdep).

    rosdep install imu_tools

Compile the stack:

    cd ~/catkin_ws
    catkin_make

### From source (ROS2)

Follow the steps from the ROS2 [Creating a
workspace](https://docs.ros.org/en/rolling/Tutorials/Workspace/Creating-A-Workspace.html)
documentation, but instead of cloning the sample repo, clone the proper branch
of this repo instead:

    git clone -b <YOUR_ROSDISTO> https://github.com/CCNYRoboticsLab/imu_tools.git


More info
---------

All nodes, topics and parameters are documented on [this repo's ROS wiki
page](https://wiki.ros.org/imu_tools).


pre-commit formatting checks
----------------------------

This repo has a [pre-commit](https://pre-commit.com/) check that runs in CI.
You can use this locally and set it up to run automatically before you commit
something. To install, use pip:

```bash
pip3 install --user pre-commit
```

To run over all the files in the repo manually:

```bash
pre-commit run -a
```

To run pre-commit automatically before committing in the local repo, install the git hooks:

```bash
pre-commit install
```

License
-------

* `imu_filter_madgwick`: currently licensed as GPL, following the original implementation

* `imu_complementary_filter`: BSD

* `rviz_imu_plugin`: BSD
