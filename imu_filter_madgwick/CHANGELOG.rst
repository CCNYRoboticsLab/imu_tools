^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imu_filter_madgwick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.4 (2017-05-22)
------------------
* Print warning if waiting for topic
  Closes `#61 <https://github.com/ccny-ros-pkg/imu_tools/issues/61>`_.
* Fix boost::lock_error on shutdown
* Contributors: Martin Günther

1.1.3 (2017-03-10)
------------------
* Return precisely normalized quaternions
  Fixes `#67 <https://github.com/ccny-ros-pkg/imu_tools/issues/67>`_ : TF_DENORMALIZED_QUATERNION warning added in TF2 0.5.14.
* Tests: Check that output quaternions are normalized
* Fixed lock so it stays in scope until end of method.
* Contributors: Jason Mercer, Martin Guenther, Martin Günther

1.1.2 (2016-09-07)
------------------
* Add missing dependency on tf2_geometry_msgs
* Contributors: Martin Guenther

1.1.1 (2016-09-07)
------------------
* Add parameter "world_frame": optionally use ENU or NED instead of NWU
  convention (from `#60 <https://github.com/ccny-ros-pkg/imu_tools/issues/60>`_;
  closes `#36 <https://github.com/ccny-ros-pkg/imu_tools/issues/36>`_)
* Add parameter "stateless" for debugging purposes: don't do any stateful
  filtering, but instead publish the orientation directly computed from the
  latest accelerometer (+ optionally magnetometer) readings alone
* Replace the (buggy) Euler-angle-based initialization routine
  (ImuFilterRos::computeRPY) by a correct transformation
  matrix based one (StatelessOrientation::computeOrientation) and make it
  available as a library function
* Refactor madgwickAHRSupdate() (pull out some functions, remove micro
  optimizations to improve readability)
* Add unit tests
* Contributors: Martin Guenther, Michael Stoll

1.1.0 (2016-04-25)
------------------

1.0.11 (2016-04-22)
-------------------
* Jade: Change default: use_magnetic_field_msg = true
* Contributors: Martin Guenther

1.0.10 (2016-04-22)
-------------------

1.0.9 (2015-10-16)
------------------

1.0.8 (2015-10-07)
------------------

1.0.7 (2015-10-07)
------------------

1.0.6 (2015-10-06)
------------------
* Split ImuFilter class into ImuFilter and ImuFilterRos in order to
  have a C++ API to the Madgwick algorithm
* Properly install header files.
* Contributors: Martin Günther, Michael Stoll

1.0.5 (2015-06-24)
------------------
* Add "~use_magnetic_field_msg" param.
  This allows the user to subscribe to the /imu/mag topic as a
  sensor_msgs/MagneticField rather than a geometry_msgs/Vector3Stamped.
  The default for now is false, which preserves the legacy behaviour via a
  separate subscriber which converts Vector3Stamped to MagneticField and
  republishes.
* Contributors: Mike Purvis, Martin Günther

1.0.4 (2015-05-06)
------------------
* update dynamic reconfigure param descriptions
* only advertise debug topics if they are used
* allow remapping of the whole imu namespace
  with this change, all topics can be remapped at once, like this:
  rosrun imu_filter_madgwick imu_filter_node imu:=my_imu
* Contributors: Martin Günther

1.0.3 (2015-01-29)
------------------
* Add std dev parameter to orientation estimate covariance matrix
* Port imu_filter_madgwick to tf2
* Switch to smart pointer
* Contributors: Paul Bovbel, Martin Günther

1.0.2 (2015-01-27)
------------------
* fix tf publishing (switch parent + child frames)
  The orientation is between a fixed inertial frame (``fixed_frame_``) and
  the frame that the IMU is mounted in (``imu_frame_``). Also,
  ``imu_msg.header.frame`` should be ``imu_frame_``, but the corresponding TF
  goes from ``fixed_frame_`` to ``imu_frame_``. This commit fixes that; for
  the ``reverse_tf`` case, it was already correct.
  Also see http://answers.ros.org/question/50870/what-frame-is-sensor_msgsimuorientation-relative-to/.
  Note that tf publishing should be enabled for debug purposes only, since we can only
  provide the orientation, not the translation.
* Add ~reverse_tf parameter for the robots which does not have IMU on root-link
* Log mag bias on startup to assist with debugging.
* add boost depends to CMakeLists
  All non-catkin things that we expose in our headers should be added to
  the DEPENDS, so that packages which depend on our package will also
  automatically link against it.
* Contributors: Martin Günther, Mike Purvis, Ryohei Ueda

1.0.1 (2014-12-10)
------------------
* add me as maintainer to package.xml
* turn mag_bias into a dynamic reconfigure param
  Also rename mag_bias/x --> mag_bias_x etc., since dynamic reconfigure
  doesn't allow slashes.
* gain and zeta already set via dynamic_reconfigure
  Reading the params explicitly is not necessary. Instead,
  dynamic_reconfigure will read them and set them as soon as we call
  config_server->setCallback().
* reconfigure server: use proper namespace
  Before, the reconfigure server used the private namespace of the nodelet
  *manager* instead of the nodelet, so the params on the parameter server
  and the ones from dynamic_reconfigure were out of sync.
* check for NaNs in magnetometer message
  Some magnetometer drivers (e.g. phidgets_drivers) output NaNs, which
  is a valid way of saying that this measurement is invalid. During
  initialization, we simply wait for the first valid message, assuming
  there will be one soon.
* magnetometer msg check: isnan() -> !isfinite()
  This catches both inf and NaN. Not sure whether sending inf in a Vector3
  message is valid (Nan is), but this doesn't hurt and is just good
  defensive programming.
* Initialize yaw from calibrated magnetometer data
  * Add magnetometer biases (mag_bias/x and mag_bias/y) for hard-iron compensation.
  * Initialize yaw orientation from magnetometer reading.
  * Add imu/rpy/raw and imu/rpy/filtered as debug topics. imu/rpy/raw can be used for computing magnetometer biases. imu/rpy/filtered topic is for user readability only.
* Contributors: Martin Günther, Shokoofeh Pourmehr

1.0.0 (2014-09-03)
------------------
* First public release
* Remove setting imu message frame to fixed/odom
* CMakeLists: remove unnecessary link_directories, LIBRARY_OUTPUT_PATH
* add missing build dependency on generated config
  This removes a racing condition from the build process.
* install nodelet xml file
  Otherwise the nodelet can't be found
* fix implementation of invSqrt()
  The old invSqrt() implementation causes the estimate to diverge under
  constant input. The problem was the line `long i = (long)&y;`, where 64
  bits are read from a 32 bit number. Thanks to @tomas-c for spotting this
  and pointing out the solution.
* catkinization of imu_tools metapackage
* fix typo: zeta -> ``zeta_``
* fix initialization of initial rotation
* gyro drift correction function added in MARG implementation
* set "zeta" as a parameter for dynamic reconfigure in the .cfg file
* add new test bag: phidgets_imu_upside_down
* add parameter publish_tf
  When the imu is used together with other packages, such as
  robot_pose_ekf, publishing the transform often interferes with those
  packages. This parameter allows to disable tf publishing.
* add some sample imu data
* more informative constant_dt message. Reverts to 0.0 on illegal param value
* imu_filter_madgwick manifest now correctly lists the package as GPL license.
* orientation is initialized from acceleration vector on first message received
* added dynamic reconfigure for gain parameter. Added better messages about constant_dt param at startup
* the tf published is now timestamped as the imu msg, and not as now(). Also added constant dt option for the imu+mag callback
* fix the transform publish -- from the fixed frame to the frame of the imu
* add a tf broadcaster with the orientation
* as per PaulKemppi: added option to set constant dt
* walchko: Needed to add namespace: std::isnan() and needed to add rosbuild_link_boost(imu_filter signals) to CMakeLists.txt
* added sebastian's name and link to the manifest
* renamed imu_filter to imu_filter_madgwick
* Contributors: Ivan Dryanovski, Martin Günther, Mike Purvis, Sameer Parekh, TUG-DESTOP, Francisco Vina, Michael Görner, Paul Kemppi, Tomas Cerskus, Kevin Walchko
