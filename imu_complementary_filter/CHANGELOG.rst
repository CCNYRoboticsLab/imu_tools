^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imu_complementary_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
