^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rviz_imu_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.2 (2020-05-25)
------------------
* Export symbols so plugin can load
* properly show/hide visualization when enabled/disabled
* Contributors: CCNY Robotics Lab, Lou Amadio, Martin Günther, v4hn

1.2.1 (2019-05-06)
------------------
* Fix includes, typos and log messages
* print ros_warn and give unit quaternion to ogre to prevent rviz crash (`#90 <https://github.com/ccny-ros-pkg/imu_tools/issues/90>`_)
* Contributors: Jackey-Huo, Martin Günther

1.2.0 (2018-05-25)
------------------

1.1.5 (2017-05-24)
------------------

1.1.4 (2017-05-22)
------------------
* Add option to display orientation in world frame (`#69 <https://github.com/ccny-ros-pkg/imu_tools/issues/69>`_)
  Per REP 145 IMU orientation is in the world frame. Rotating the
  orientation data to transform into the sensor frame results in strange
  behavior, such as double-rotation of orientation on a robot. Provide an
  option to display orientation in the world frame, and enable it by
  default. Continue to translate the position of the data to the sensor
  frame.
* Contributors: C. Andy Martin

1.1.3 (2017-03-10)
------------------

1.1.2 (2016-09-07)
------------------

1.1.1 (2016-09-07)
------------------

1.1.0 (2016-04-25)
------------------
* Add qt5 dependencies to rviz_imu_plugin package.xml
  This fixes the compilation errors on Kinetic for Debian Jessie.
* Contributors: Martin Guenther

1.0.11 (2016-04-22)
-------------------

1.0.10 (2016-04-22)
-------------------
* Support qt4/qt5 using rviz's exported qt version
  Closes `#58 <https://github.com/ccny-ros-pkg/imu_tools/issues/58>`_ .
  This fixes the build on Kinetic, where only Qt5 is available, and
  is backwards compatible with Qt4 for Indigo.
* Contributors: Martin Guenther

1.0.9 (2015-10-16)
------------------

1.0.8 (2015-10-07)
------------------

1.0.7 (2015-10-07)
------------------

1.0.6 (2015-10-06)
------------------

1.0.5 (2015-06-24)
------------------

1.0.4 (2015-05-06)
------------------

1.0.3 (2015-01-29)
------------------

1.0.2 (2015-01-27)
------------------

1.0.1 (2014-12-10)
------------------
* add me as maintainer to package.xml
* Contributors: Martin Günther

1.0.0 (2014-09-03)
------------------
* First public release
* Contributors: Ivan Dryanovski, Martin Günther, Davide Tateo, Francisco Vina, Lorenzo Riano
