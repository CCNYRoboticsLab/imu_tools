^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imu_complementary_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.3 (2022-12-07)
------------------
* complementary: Build shared library
  See `#172 <https://github.com/CCNYRoboticsLab/imu_tools/issues/172>`_.
* Update CMakeLists to use targets
* Remove node\_ prefix. (`#163 <https://github.com/CCNYRoboticsLab/imu_tools/issues/163>`_)
* Contributors: Martin Günther, Max Polzin

2.1.2 (2022-07-14)
------------------

2.1.1 (2022-05-24)
------------------
* Add missing build dependency to package.xml. (`#161 <https://github.com/CCNYRoboticsLab/imu_tools/issues/161>`_)
* Contributors: Martin Günther, Steven! Ragnarök

2.1.0 (2022-05-02)
------------------
* complementary: Add missing dependency on geometry_msgs
* Contributors: Martin Günther

2.0.0 (2022-04-12)
------------------
* Initial release into ROS2 foxy, galactic and rolling
* Fix gcc warnings + clang-tidy suggestions
* Fix CMakeLists
* Reformat python code using black
* Manually reformat licenses + defines
* Reformat everything using clang-format
* Fix trailing whitespace
* Add launch directory to CMakeLists.txt (`#146 <https://github.com/CCNYRoboticsLab/imu_tools/issues/146>`_)
* Port imu_complementary_filter to ROS2 (`#138 <https://github.com/CCNYRoboticsLab/imu_tools/issues/138>`_)
* Madgwick for eloquent (`#110 <https://github.com/CCNYRoboticsLab/imu_tools/issues/110>`_)
* Contributors: Guido Sanchez, Martin Günther, Maximilian Schik, tgreier

1.2.2 (2020-05-25)
------------------
* fix install path & boost linkage issues
* Contributors: Martin Günther, Sean Yen

1.2.1 (2019-05-06)
------------------
* Remove junk xml (`#93 <https://github.com/ccny-ros-pkg/imu_tools/issues/93>`_)
* Fix C++14 builds (`#89 <https://github.com/ccny-ros-pkg/imu_tools/issues/89>`_)
* Contributors: David V. Lu!!, Paul Bovbel

1.2.0 (2018-05-25)
------------------
* Add std dev parameter to orientation estimate from filter (`#85 <https://github.com/ccny-ros-pkg/imu_tools/issues/85>`_)
  Similar to `#41 <https://github.com/ccny-ros-pkg/imu_tools/issues/41>`_, but not using dynamic_reconfigure as not implemented for complementary filter
* Contributors: Stefan Kohlbrecher

1.1.5 (2017-05-24)
------------------

1.1.4 (2017-05-22)
------------------

1.1.3 (2017-03-10)
------------------
* complementary_filter: move const initializations out of header
  Initialization of static consts other than int (here: float) inside the
  class declaration is not permitted in C++. It works in gcc (due to a
  non-standard extension), but throws an error in C++11.
* Contributors: Martin Guenther

1.1.2 (2016-09-07)
------------------

1.1.1 (2016-09-07)
------------------

1.1.0 (2016-04-25)
------------------

1.0.11 (2016-04-22)
-------------------

1.0.10 (2016-04-22)
-------------------
* Remove Eigen dependency
  Eigen is not actually used anywhere. Thanks @asimay!
* Removed main function from shared library
* Contributors: Martin Guenther, Matthias Nieuwenhuisen

1.0.9 (2015-10-16)
------------------
* complementary: Add Eigen dependency
  Fixes `#54 <https://github.com/ccny-ros-pkg/imu_tools/issues/54>`_.
* Contributors: Martin Günther

1.0.8 (2015-10-07)
------------------

1.0.7 (2015-10-07)
------------------
* Allow remapping imu namespace
* Publish RPY as Vector3Stamped
* Add params: constant_dt, publish_tf, reverse_tf, publish_debug_topics
* Use MagneticField instead of Vector3
* Contributors: Martin Günther

1.0.6 (2015-10-06)
------------------
* Add new package: imu_complementary_filter
* Contributors: Roberto G. Valentini, Martin Günther, Michael Görner

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

1.0.0 (2014-11-28)
------------------
