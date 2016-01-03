/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "imu_filter_madgwick/imu_filter_ros.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

ImuFilterRos::ImuFilterRos(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  initialized_(false)
{
  ROS_INFO ("Starting ImuFilter");

  // *** Set default parameters

  const std::string mag_subscribe_topic_name_default = "imu/mag";
  const std::string mag_publish_topic_name_default = "imu/mag_corrected"; // currently not used, should be used for publishing corrected magnetic fields
  const std::string imu_subscribe_topic_name_default = "imu/data_raw";
  const std::string imu_publish_topic_name_default = "imu/data";
  std::vector<double> zero_3 (3,0.0);
  std::vector<double> ones_3 (3,1.0);

  // **** get paramters

  if (!nh_private_.getParam ("use_mag", use_mag_))
   use_mag_ = true;
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
   publish_tf_ = true;
  if (!nh_private_.getParam ("reverse_tf", reverse_tf_))
   reverse_tf_ = false;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
   fixed_frame_ = "odom";
  if (!nh_private_.getParam ("constant_dt", constant_dt_))
    constant_dt_ = 0.0;
  if (!nh_private_.getParam ("publish_debug_topics", publish_debug_topics_))
    publish_debug_topics_= false;

  if (!nh_private.hasParam ("gyro_offset") || !nh_private.hasParam ("acc_offset") || !nh_private.hasParam ("acc_scaling")){
    ROS_WARN ("No configuration parameters found ...");
  }
  else{
    ROS_INFO ("the configuration parameters for the IMU are");
    nh_private.param ("gyro_offset", gyro_offset_, zero_3);
    nh_private.param ("acc_offset", acc_offset_, zero_3);
    nh_private.param ("acc_scaling", acc_scaling_, ones_3);
//    ROS_INFO (gyro_offset_);
//    ROS_INFO (acc_offset_);
//    ROS_INFO (acc_scaling_);
  }

  nh_private.param("mag_subscribe_topic_name", mag_topic_sub_, mag_subscribe_topic_name_default);
  nh_private.param("mag_publish_topic_name", mag_topic_pub_, mag_publish_topic_name_default);
  nh_private.param("imu_subscribe_topic_name", imu_topic_sub_, imu_subscribe_topic_name_default);
  nh_private.param("imu_publish_topic_name", imu_topic_pub_, imu_publish_topic_name_default);

  // For ROS Jade, make this default to true.
  if (!nh_private_.getParam ("use_magnetic_field_msg", use_magnetic_field_msg_))
  {
      if (use_mag_)
      {
          ROS_WARN("Deprecation Warning: The parameter use_magnetic_field_msg was not set, default is 'false'.");
          ROS_WARN("Starting with ROS Jade, use_magnetic_field_msg will default to 'true'!");
      }
      use_magnetic_field_msg_ = false;
  }

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    ROS_FATAL("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
    constant_dt_ = 0.0;
  }

  // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
  // otherwise, it will be constant
  if (constant_dt_ == 0.0)
    ROS_INFO("Using dt computed from message headers");
  else
    ROS_INFO("Using constant dt of %f sec", constant_dt_);

  // **** register dynamic reconfigure
  config_server_.reset(new FilterConfigServer(nh_private_));
  FilterConfigServer::CallbackType f = boost::bind(&ImuFilterRos::reconfigCallback, this, _1, _2);
  config_server_->setCallback(f);

  // **** register publishers
//  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(
//    ros::names::resolve("imu") + "/data", 5);

    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_pub_, 5);

  if (publish_debug_topics_)
  {
    rpy_filtered_debug_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      ros::names::resolve("imu") + "/rpy/filtered", 5);

    rpy_raw_debug_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
      ros::names::resolve("imu") + "/rpy/raw", 5);
  }

  // **** register subscribers
  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;

  imu_subscriber_.reset(new ImuSubscriber(
    nh_, imu_topic_sub_, queue_size));

  if (use_mag_)
  {
    if (use_magnetic_field_msg_)
    {
      mag_subscriber_.reset(new MagSubscriber(
        nh_, mag_topic_sub_, queue_size));
    }
    else
    {
      mag_subscriber_.reset(new MagSubscriber(
        nh_, ros::names::resolve("imu") + "/magnetic_field", queue_size));

      // Initialize the shim to support republishing Vector3Stamped messages from /mag as MagneticField
      // messages on the /magnetic_field topic.
      mag_republisher_ = nh_.advertise<MagMsg>(
        ros::names::resolve("imu") + "/magnetic_field", 5);
      vector_mag_subscriber_.reset(new MagVectorSubscriber(
        nh_, mag_topic_sub_, queue_size));
      vector_mag_subscriber_->registerCallback(&ImuFilterRos::imuMagVectorCallback, this);
    }

    sync_.reset(new Synchronizer(
      SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
    sync_->registerCallback(boost::bind(&ImuFilterRos::imuMagCallback, this, _1, _2));
  }
  else
  {
    imu_subscriber_->registerCallback(&ImuFilterRos::imuCallback, this);
  }
}

ImuFilterRos::~ImuFilterRos()
{
  ROS_INFO ("Destroying ImuFilter");
}

void ImuFilterRos::imuCallback(const ImuMsg::ConstPtr& imu_msg_raw)
{
  boost::mutex::scoped_lock(mutex_);

  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration;

  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  if (!initialized_)
  {
    // initialize roll/pitch orientation from acc. vector
    double sign = copysignf(1.0, lin_acc.z);
    double roll = atan2(lin_acc.y, sign * sqrt(lin_acc.x*lin_acc.x + lin_acc.z*lin_acc.z));
    double pitch = -atan2(lin_acc.x, sqrt(lin_acc.y*lin_acc.y + lin_acc.z*lin_acc.z));
    double yaw = 0.0;

    tf2::Quaternion init_q;
    init_q.setRPY(roll, pitch, yaw);

    filter_.setOrientation(init_q.getW(), init_q.getX(), init_q.getY(), init_q.getZ());

    // initialize time
    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - last_time_).toSec();

  last_time_ = time;

  filter_.madgwickAHRSupdateIMU(
    ang_vel.x, ang_vel.y, ang_vel.z,
    lin_acc.x, lin_acc.y, lin_acc.z,
    dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);
}

void ImuFilterRos::imuMagCallback(
  const ImuMsg::ConstPtr& imu_msg_raw,
  const MagMsg::ConstPtr& mag_msg)
{
  boost::mutex::scoped_lock(mutex_);

  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
  const geometry_msgs::Vector3& mag_fld = mag_msg->magnetic_field;

  geometry_msgs::Vector3 lin_acc_corr;
  geometry_msgs::Vector3 ang_vel_corr;


  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  /*** Compensate for hard iron ***/
  double mx = mag_fld.x - mag_bias_.x;
  double my = mag_fld.y - mag_bias_.y;
  double mz = mag_fld.z - mag_bias_.z;

  /*** Compensate accelerometer ***/
  lin_acc_corr.x = (lin_acc.x-acc_offset_[0])*acc_scaling_[0];
  lin_acc_corr.y = (lin_acc.y-acc_offset_[1])*acc_scaling_[1];
  lin_acc_corr.z = (lin_acc.z-acc_offset_[2])*acc_scaling_[2];

  /*** Compensate gyroscopes ***/
  ang_vel_corr.x = ang_vel.x - gyro_offset_[0];
  ang_vel_corr.y = ang_vel.y - gyro_offset_[1];
  ang_vel_corr.z = ang_vel.z - gyro_offset_[2];

  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;

  if (!initialized_)
  {
    // wait for mag message without NaN / inf
    if(!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) || !std::isfinite(mag_fld.z))
    {
      return;
    }

    computeRPY(
      lin_acc_corr.x, lin_acc_corr.y, lin_acc_corr.z,
      mx, my, mz,
      roll, pitch, yaw);

    tf2::Quaternion init_q;
    init_q.setRPY(roll, pitch, yaw);

    filter_.setOrientation(init_q.getW(), init_q.getX(), init_q.getY(), init_q.getZ());

    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - last_time_).toSec();

  last_time_ = time;

  filter_.madgwickAHRSupdate(
    ang_vel_corr.x, ang_vel_corr.y, ang_vel_corr.z,
    lin_acc_corr.x, lin_acc_corr.y, lin_acc_corr.z,
    mx, my, mz,
    dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);

  if(publish_debug_topics_ && std::isfinite(mx) && std::isfinite(my) && std::isfinite(mz))
  {
    computeRPY(
      lin_acc_corr.x, lin_acc_corr.y, lin_acc_corr.z,
      mx, my, mz,
      roll, pitch, yaw);

    publishRawMsg(time, roll, pitch, yaw);
  }
}

void ImuFilterRos::publishTransform(const ImuMsg::ConstPtr& imu_msg_raw)
{
  double q0,q1,q2,q3;
  filter_.getOrientation(q0,q1,q2,q3);
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = imu_msg_raw->header.stamp;
  if (reverse_tf_)
  {
    transform.header.frame_id = imu_frame_;
    transform.child_frame_id = fixed_frame_;
    transform.transform.rotation.w = q0;
    transform.transform.rotation.x = -q1;
    transform.transform.rotation.y = -q2;
    transform.transform.rotation.z = -q3;
  }
  else {
    transform.header.frame_id = fixed_frame_;
    transform.child_frame_id = imu_frame_;
    transform.transform.rotation.w = q0;
    transform.transform.rotation.x = q1;
    transform.transform.rotation.y = q2;
    transform.transform.rotation.z = q3;
  }
  tf_broadcaster_.sendTransform(transform);

}

void ImuFilterRos::publishFilteredMsg(const ImuMsg::ConstPtr& imu_msg_raw)
{
  double q0,q1,q2,q3;
  filter_.getOrientation(q0,q1,q2,q3);

  // create and publish filtered IMU message
  boost::shared_ptr<ImuMsg> imu_msg =
    boost::make_shared<ImuMsg>(*imu_msg_raw);

  imu_msg->orientation.w = q0;
  imu_msg->orientation.x = q1;
  imu_msg->orientation.y = q2;
  imu_msg->orientation.z = q3;

  imu_msg->orientation_covariance[0] = orientation_variance_;
  imu_msg->orientation_covariance[1] = 0.0;
  imu_msg->orientation_covariance[2] = 0.0;
  imu_msg->orientation_covariance[3] = 0.0;
  imu_msg->orientation_covariance[4] = orientation_variance_;
  imu_msg->orientation_covariance[5] = 0.0;
  imu_msg->orientation_covariance[6] = 0.0;
  imu_msg->orientation_covariance[7] = 0.0;
  imu_msg->orientation_covariance[8] = orientation_variance_;

  imu_publisher_.publish(imu_msg);

  if(publish_debug_topics_)
  {
    geometry_msgs::Vector3Stamped rpy;
    tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

    rpy.header = imu_msg_raw->header;
    rpy_filtered_debug_publisher_.publish(rpy);
  }
}

void ImuFilterRos::publishRawMsg(const ros::Time& t,
  float roll, float pitch, float yaw)
{
  geometry_msgs::Vector3Stamped rpy;
  rpy.vector.x = roll;
  rpy.vector.y = pitch;
  rpy.vector.z = yaw ;
  rpy.header.stamp = t;
  rpy.header.frame_id = imu_frame_;
  rpy_raw_debug_publisher_.publish(rpy);
}

void ImuFilterRos::computeRPY(
  float ax, float ay, float az,
  float mx, float my, float mz,
  float& roll, float& pitch, float& yaw)
{
  // initialize roll/pitch orientation from acc. vector.
  double sign = copysignf(1.0, az);
  roll = atan2(ay, sign * sqrt(ax*ax + az*az));
  pitch = -atan2(ax, sqrt(ay*ay + az*az));
  double cos_roll = cos(roll);
  double sin_roll = sin(roll);
  double cos_pitch = cos(pitch);
  double sin_pitch = sin(pitch);

  // initialize yaw orientation from magnetometer data.
  /***  From: http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf (equation 22). ***/
  double head_x = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
  double head_y = my * cos_roll - mz * sin_roll;
  yaw = atan2(-head_y, head_x);
}

void ImuFilterRos::reconfigCallback(FilterConfig& config, uint32_t level)
{
  double gain, zeta;
  boost::mutex::scoped_lock(mutex_);
  gain = config.gain;
  zeta = config.zeta;
  filter_.setAlgorithmGain(gain);
  filter_.setDriftBiasGain(zeta);
  ROS_INFO("Imu filter gain set to %f", gain);
  ROS_INFO("Gyro drift bias set to %f", zeta);
  mag_bias_.x = config.mag_bias_x;
  mag_bias_.y = config.mag_bias_y;
  mag_bias_.z = config.mag_bias_z;
  orientation_variance_ = config.orientation_stddev * config.orientation_stddev;
  ROS_INFO("Magnetometer bias values: %f %f %f", mag_bias_.x, mag_bias_.y, mag_bias_.z);
}

void ImuFilterRos::imuMagVectorCallback(const MagVectorMsg::ConstPtr& mag_vector_msg)
{
  MagMsg mag_msg;
  mag_msg.header = mag_vector_msg->header;
  mag_msg.magnetic_field = mag_vector_msg->vector;
  // leaving mag_msg.magnetic_field_covariance set to all zeros (= "covariance unknown")
  mag_republisher_.publish(mag_msg);
}
