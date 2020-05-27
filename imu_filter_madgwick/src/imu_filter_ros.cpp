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
#include <chrono>

#include "imu_filter_madgwick/imu_filter_ros.h"
#include "imu_filter_madgwick/stateless_orientation.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;
using namespace rclcpp;
using namespace std::placeholders;

ImuFilterMadgwickRos::ImuFilterMadgwickRos(const rclcpp::NodeOptions &options)
    : Node("imu_filter_madgwick", options), tf_broadcaster_(this), initialized_(false) {
  RCLCPP_INFO (get_logger(), "Starting ImuFilter");

  // **** get paramters
  declare_parameter("stateless", false);
  get_parameter("stateless", stateless_);
  declare_parameter("use_mag", true);
  get_parameter("use_mag", use_mag_);
  declare_parameter("publish_tf", true);
  get_parameter("publish_tf", publish_tf_);
  declare_parameter("reverse_tf", false);
  get_parameter("reverse_tf", reverse_tf_);
  declare_parameter("fixed_frame", "odom");
  get_parameter("fixed_frame", fixed_frame_);
  declare_parameter("constant_dt", 0.0);
  get_parameter("constant_dt", constant_dt_);
  declare_parameter("remove_gravity_vector", false);
  get_parameter("remove_gravity_vector", remove_gravity_vector_);
  declare_parameter("publish_debug_topics", false);
  get_parameter("publish_debug_topics", publish_debug_topics_);

  RCLCPP_ERROR_STREAM(get_logger(), "use_mag: " << use_mag_ << " stateless: " << stateless_);

  std::string world_frame;
  declare_parameter("world_frame", "enu");
  get_parameter("world_frame", world_frame);
  if (world_frame == "ned") {
    world_frame_ = WorldFrame::NED;
  } else if (world_frame == "nwu") {
    world_frame_ = WorldFrame::NWU;
  } else if (world_frame == "enu") {
    world_frame_ = WorldFrame::ENU;
  } else {
    RCLCPP_ERROR(get_logger(), "The parameter world_frame was set to invalid value '%s'.",
                 world_frame.c_str());
    RCLCPP_ERROR(get_logger(), "Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
    world_frame_ = WorldFrame::ENU;
  }
  filter_.setWorldFrame(world_frame_);

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    RCLCPP_ERROR(get_logger(), "constant_dt parameter is %f, must be >= 0.0. Setting to 0.0",
                 constant_dt_);
    constant_dt_ = 0.0;
  }

  // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
  // otherwise, it will be constant
  if (constant_dt_ == 0.0) {
    RCLCPP_ERROR(get_logger(), "Using dt computed from message headers");
  } else {
    RCLCPP_ERROR(get_logger(), "Using constant dt of %f sec", constant_dt_);
  }

  if (remove_gravity_vector_) {
    RCLCPP_ERROR(get_logger(), "The gravity vector will be removed from the acceleration");
  } else {
    RCLCPP_ERROR(get_logger(), "The gravity vector is kept in the IMU message.");
  }

  // TODO: Add reconfigure parameter descriptions.
  // gen.add("gain", double_t, 0, "Gain of the filter. Higher values lead to faster convergence but
  // more noise. Lower values lead to slower convergence but smoother signal.", 0.1, 0.0, 1.0)
  // gen.add("zeta", double_t, 0, "Gyro drift gain (approx. rad/s).", 0, -1.0, 1.0)
  // gen.add("mag_bias_x", double_t, 0, "Magnetometer bias (hard iron correction), x component.", 0,
  // -10.0, 10.0) gen.add("mag_bias_y", double_t, 0, "Magnetometer bias (hard iron correction), y
  // component.", 0, -10.0, 10.0) gen.add("mag_bias_z", double_t, 0, "Magnetometer bias (hard iron
  // correction), z component.", 0, -10.0, 10.0) gen.add("orientation_stddev", double_t, 0,"Standard
  // deviation of the orientation estimate.", 0, 0, 1.0)

  double gain;
  declare_parameter("gain", 0.1);
  get_parameter("gain", gain);
  double zeta;
  declare_parameter("zeta", 0.0);
  get_parameter("zeta", zeta);
  double mag_bias_x;
  declare_parameter("mag_bias_x", 0.0);
  get_parameter("mag_bias_x", mag_bias_x);
  double mag_bias_y;
  declare_parameter("mag_bias_y", 0.0);
  get_parameter("mag_bias_y", mag_bias_y);
  double mag_bias_z;
  declare_parameter("mag_bias_z", 0.0);
  get_parameter("mag_bias_z", mag_bias_z);
  double orientation_stddev;
  declare_parameter("orientation_stddev", 0.0);
  get_parameter("orientation_stddev", orientation_stddev);
  filter_.setAlgorithmGain(gain);
  filter_.setDriftBiasGain(zeta);
  RCLCPP_INFO(get_logger(), "Imu filter gain set to %f", gain);
  RCLCPP_INFO(get_logger(), "Gyro drift bias set to %f", zeta);
  mag_bias_.x = mag_bias_x;
  mag_bias_.y = mag_bias_y;
  mag_bias_.z = mag_bias_z;
  orientation_variance_ = orientation_stddev * orientation_stddev;
  RCLCPP_INFO(get_logger(), "Magnetometer bias values: %f %f %f", mag_bias_.x, mag_bias_.y,
              mag_bias_.z);

  // **** register dynamic reconfigure
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this->get_node_base_interface(), this->get_node_topics_interface(),
      this->get_node_graph_interface(), this->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
      std::bind(&ImuFilterMadgwickRos::reconfigCallback, this, _1));

  // **** register publishers
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("~/imu/data", 5);
  if (publish_debug_topics_) {
    rpy_filtered_debug_publisher_ =
        create_publisher<geometry_msgs::msg::Vector3Stamped>("~/imu/rpy/filtered", 5);

    rpy_raw_debug_publisher_ =
        create_publisher<geometry_msgs::msg::Vector3Stamped>("~/imu/rpy/raw", 5);
  }

  // **** register subscribers
  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  const int queue_size = 5;
  rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
  imu_subscriber_.reset(new ImuSubscriber(this, "~/imu/data_raw", qos));

  if (use_mag_)
  {
    mag_subscriber_.reset(new MagSubscriber(this, "~/imu/mag", qos));

    sync_.reset(new Synchronizer(SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
    sync_->registerCallback(&ImuFilterMadgwickRos::imuMagCallback, this);
  }
  else
  {
    imu_subscriber_->registerCallback(&ImuFilterMadgwickRos::imuCallback, this);
  }

  check_topics_timer_ = create_wall_timer(
      std::chrono::seconds(10), std::bind(&ImuFilterMadgwickRos::checkTopicsTimerCallback, this));
}

void ImuFilterMadgwickRos::imuCallback(const ImuMsg::SharedPtr imu_msg_raw) {
  std::lock_guard<std::mutex> lock(mutex_);

  const geometry_msgs::msg::Vector3 &ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Vector3 &lin_acc = imu_msg_raw->linear_acceleration;

  rclcpp::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  if (!initialized_ || stateless_) 
  {
    geometry_msgs::msg::Quaternion init_q;
    if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc, init_q))
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 5.0, "The IMU seems to be in free fall, cannot determine gravity direction!");
      return;
    }
    filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
  }

  if (!initialized_) 
  {
    RCLCPP_INFO(get_logger(), "First IMU message received.");
    check_topics_timer_->cancel();

    // initialize time
    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
  {
    dt = (time - last_time_).seconds();
    if (0 == time.nanoseconds()) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), steady_clock, 5.0, "The IMU message time stamp is zero, and the parameter constant_dt is not set!" <<
                                    " The filter will not update the orientation.");
    }
  }

  last_time_ = time;

  if (!stateless_)
    filter_.madgwickAHRSupdateIMU(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);
}

void ImuFilterMadgwickRos::imuMagCallback(const ImuMsg::SharedPtr imu_msg_raw,
                                          const MagMsg::SharedPtr mag_msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  const geometry_msgs::msg::Vector3 &ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::msg::Vector3 &lin_acc = imu_msg_raw->linear_acceleration;
  const geometry_msgs::msg::Vector3 &mag_fld = mag_msg->magnetic_field;

  rclcpp::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  /*** Compensate for hard iron ***/
  geometry_msgs::msg::Vector3 mag_compensated;
  mag_compensated.x = mag_fld.x - mag_bias_.x;
  mag_compensated.y = mag_fld.y - mag_bias_.y;
  mag_compensated.z = mag_fld.z - mag_bias_.z;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  if (!initialized_ || stateless_) 
  {
    // wait for mag message without NaN / inf
    if(!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) || !std::isfinite(mag_fld.z))
    {
      return;
    }

    geometry_msgs::msg::Quaternion init_q;
    if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, init_q))
    {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 5.0, "The IMU seems to be in free fall or close to magnetic north pole, cannot determine gravity direction!");
      return;
    }
    filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
  }

  if (!initialized_)
  {
    RCLCPP_INFO(get_logger(), "First pair of IMU and magnetometer messages received.");
    check_topics_timer_->cancel();

    // initialize time
    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
  {
    dt = (time - last_time_).seconds();
    if (0 == time.nanoseconds()) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), steady_clock, 5.0, "The IMU message time stamp is zero, and the parameter constant_dt is not set!" <<
                                    " The filter will not update the orientation.");
    }
  }

  last_time_ = time;

  if (!stateless_) 
  {
    filter_.madgwickAHRSupdate(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      mag_compensated.x, mag_compensated.y, mag_compensated.z,
      dt);
  }

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
  {
    publishTransform(imu_msg_raw);
  }

  if(publish_debug_topics_)
  {
    geometry_msgs::msg::Quaternion orientation;
    if (StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, orientation))
    {
      tf2::Matrix3x3(tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)).getRPY(roll, pitch, yaw, 0);
      publishRawMsg(time, roll, pitch, yaw);
    }
  }
}

void ImuFilterMadgwickRos::publishTransform(const ImuMsg::SharedPtr imu_msg_raw) {
  double q0,q1,q2,q3;
  filter_.getOrientation(q0,q1,q2,q3);
  geometry_msgs::msg::TransformStamped transform;
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

void ImuFilterMadgwickRos::publishFilteredMsg(const ImuMsg::SharedPtr imu_msg_raw) {
  double q0,q1,q2,q3;
  filter_.getOrientation(q0,q1,q2,q3);

  // create and publish filtered IMU message
  ImuMsg imu_msg = *imu_msg_raw;

  imu_msg.orientation.w = q0;
  imu_msg.orientation.x = q1;
  imu_msg.orientation.y = q2;
  imu_msg.orientation.z = q3;

  imu_msg.orientation_covariance[0] = orientation_variance_;
  imu_msg.orientation_covariance[1] = 0.0;
  imu_msg.orientation_covariance[2] = 0.0;
  imu_msg.orientation_covariance[3] = 0.0;
  imu_msg.orientation_covariance[4] = orientation_variance_;
  imu_msg.orientation_covariance[5] = 0.0;
  imu_msg.orientation_covariance[6] = 0.0;
  imu_msg.orientation_covariance[7] = 0.0;
  imu_msg.orientation_covariance[8] = orientation_variance_;

  imu_publisher_->publish(imu_msg);

  if(publish_debug_topics_)
  {
    geometry_msgs::msg::Vector3Stamped rpy;
    tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0)).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

    rpy.header = imu_msg_raw->header;
    rpy_filtered_debug_publisher_->publish(rpy);
  }
}

void ImuFilterMadgwickRos::publishRawMsg(const rclcpp::Time &t, 
  float roll, float pitch, float yaw)
{
  geometry_msgs::msg::Vector3Stamped rpy;
  rpy.vector.x = roll;
  rpy.vector.y = pitch;
  rpy.vector.z = yaw;
  rpy.header.stamp = t;
  rpy.header.frame_id = imu_frame_;
  rpy_raw_debug_publisher_->publish(rpy);
}

void ImuFilterMadgwickRos::reconfigCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  double gain, zeta;
  std::lock_guard<std::mutex> lock(mutex_);

  for (auto &changed_parameter : event->changed_parameters)
  {
    const auto &type = changed_parameter.value.type;
    const auto &name = changed_parameter.name;
    const auto &value = changed_parameter.value;

    if (type == ParameterType::PARAMETER_DOUBLE)
    {
      if (name == "gain") {
        gain = value.double_value;
        filter_.setAlgorithmGain(gain);
        RCLCPP_INFO(get_logger(), "Imu filter gain set to %f", gain);
      } else if (name == "zeta") {
        zeta = value.double_value;
        filter_.setDriftBiasGain(zeta);
        RCLCPP_INFO(get_logger(), "Gyro drift bias set to %f", zeta);
      } else if (name == "mag_bias_x") {
        mag_bias_.x = value.double_value;
      } else if (name == "mag_bias_y") {
        mag_bias_.y = value.double_value;
      } else if (name == "mag_bias_z") {
        mag_bias_.z = value.double_value;
      } else if (name == "orientation_stddev") {
        double orientation_stddev = value.double_value;
        orientation_variance_ = orientation_stddev * orientation_stddev;
      }
    }
  }

  RCLCPP_INFO(get_logger(), "Magnetometer bias values: %f %f %f", mag_bias_.x, mag_bias_.y,
              mag_bias_.z);
}

void ImuFilterMadgwickRos::checkTopicsTimerCallback()
{
  if (use_mag_)
    RCLCPP_WARN_STREAM(get_logger(), "Still waiting for data on topics "
                                         << "~/imu/data_raw"
                                         << " and "
                                         << "~/imu/mag"
                                         << "...");
  else
    RCLCPP_WARN_STREAM(get_logger(), "Still waiting for data on topic "
                                         << "~/imu/data_raw"
                                         << "...");
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ImuFilterMadgwickRos)