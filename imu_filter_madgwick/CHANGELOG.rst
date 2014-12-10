^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package imu_filter_madgwick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
