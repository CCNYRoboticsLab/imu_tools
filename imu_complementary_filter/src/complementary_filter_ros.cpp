/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

  @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived from
        this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include "imu_complementary_filter/complementary_filter_ros.h"

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace imu_tools {

ComplementaryFilterROS::ComplementaryFilterROS()
    : Node("ComplementaryFilterROS"),
      tf_broadcaster_(this),
      initialized_filter_(false)
{
    RCLCPP_INFO(this->get_logger(), "Starting ComplementaryFilterROS");
    initializeParams();

    int queue_size = 5;

    // Register publishers:
    // TODO: Check why ros::names::resolve is need here
    imu_publisher_ = this->create_publisher<ImuMsg>("imu/data", queue_size);

    if (publish_debug_topics_)
    {
        rpy_publisher_ =
            this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "imu/rpy/filtered", queue_size);

        if (filter_.getDoBiasEstimation())
        {
            state_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
                "/imu/steady_state", queue_size);
        }
    }

    // Register IMU raw data subscriber.
    imu_subscriber_.reset(new ImuSubscriber(this, "/imu/data_raw"));

    // Register magnetic data subscriber.
    if (use_mag_)
    {
        mag_subscriber_.reset(new MagSubscriber(this, "/imu/mag"));

        sync_.reset(new Synchronizer(SyncPolicy(queue_size), *imu_subscriber_,
                                     *mag_subscriber_));
        sync_->registerCallback(&ComplementaryFilterROS::imuMagCallback, this);
    } else
    {
        imu_subscriber_->registerCallback(&ComplementaryFilterROS::imuCallback,
                                          this);
    }
}

ComplementaryFilterROS::~ComplementaryFilterROS()
{
    RCLCPP_INFO(this->get_logger(), "Destroying ComplementaryFilterROS");
}

void ComplementaryFilterROS::initializeParams()
{
    double gain_acc;
    double gain_mag;
    bool do_bias_estimation;
    double bias_alpha;
    bool do_adaptive_gain;
    double orientation_stddev;

    fixed_frame_ = this->declare_parameter<std::string>("fixed_frame", "odom");
    use_mag_ = this->declare_parameter<bool>("use_mag", false);
    publish_tf_ = this->declare_parameter<bool>("publish_tf", false);
    reverse_tf_ = this->declare_parameter<bool>("reverse_tf", false);
    constant_dt_ = this->declare_parameter<double>("constant_dt", 0.0);
    publish_debug_topics_ =
        this->declare_parameter<bool>("publish_debug_topics", false);
    gain_acc = this->declare_parameter<double>("gain_acc", 0.01);
    gain_mag = this->declare_parameter<double>("gain_mag", 0.01);
    do_bias_estimation =
        this->declare_parameter<bool>("do_bias_estimation", true);
    bias_alpha = this->declare_parameter<double>("bias_alpha", 0.01);
    do_adaptive_gain = this->declare_parameter<bool>("do_adaptive_gain", true);
    orientation_stddev =
        this->declare_parameter<double>("orientation_stddev", 0.0);
    orientation_variance_ = orientation_stddev * orientation_stddev;

    filter_.setDoBiasEstimation(do_bias_estimation);
    filter_.setDoAdaptiveGain(do_adaptive_gain);

    if (!filter_.setGainAcc(gain_acc))
        RCLCPP_WARN(this->get_logger(),
                    "Invalid gain_acc passed to ComplementaryFilter.");
    if (use_mag_)
    {
        if (!filter_.setGainMag(gain_mag))
            RCLCPP_WARN(this->get_logger(),
                        "Invalid gain_mag passed to ComplementaryFilter.");
    }
    if (do_bias_estimation)
    {
        if (!filter_.setBiasAlpha(bias_alpha))
            RCLCPP_WARN(this->get_logger(),
                        "Invalid bias_alpha passed to ComplementaryFilter.");
    }

    // check for illegal constant_dt values
    if (constant_dt_ < 0.0)
    {
        // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
        // otherwise, it will be constant
        RCLCPP_WARN(
            this->get_logger(),
            "constant_dt parameter is %f, must be >= 0.0. Setting to 0.0",
            constant_dt_);
        constant_dt_ = 0.0;
    }
}

void ComplementaryFilterROS::imuCallback(ImuMsg::ConstSharedPtr imu_msg_raw)
{
    const geometry_msgs::msg::Vector3 &a = imu_msg_raw->linear_acceleration;
    const geometry_msgs::msg::Vector3 &w = imu_msg_raw->angular_velocity;
    const rclcpp::Time &time = imu_msg_raw->header.stamp;

    // Initialize.
    if (!initialized_filter_)
    {
        time_prev_ = time;
        initialized_filter_ = true;
        return;
    }

    // determine dt: either constant, or from IMU timestamp
    double dt;
    if (constant_dt_ > 0.0)
        dt = constant_dt_;
    else
        dt = (time - time_prev_).nanoseconds() * 1e-9;

    time_prev_ = time;

    // Update the filter.
    filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);

    // Publish state.
    publish(imu_msg_raw);
}

void ComplementaryFilterROS::imuMagCallback(ImuMsg::ConstSharedPtr imu_msg_raw,
                                            MagMsg::ConstSharedPtr mag_msg)
{
    const geometry_msgs::msg::Vector3 &a = imu_msg_raw->linear_acceleration;
    const geometry_msgs::msg::Vector3 &w = imu_msg_raw->angular_velocity;
    const geometry_msgs::msg::Vector3 &m = mag_msg->magnetic_field;
    const rclcpp::Time &time = imu_msg_raw->header.stamp;

    // Initialize.
    if (!initialized_filter_)
    {
        time_prev_ = time;
        initialized_filter_ = true;
        return;
    }

    // Calculate dt.
    double dt = (time - time_prev_).nanoseconds() * 1e-9;
    time_prev_ = time;
    // ros::Time t_in, t_out;
    // t_in = ros::Time::now();
    // Update the filter.
    if (std::isnan(m.x) || std::isnan(m.y) || std::isnan(m.z))
        filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, dt);
    else
        filter_.update(a.x, a.y, a.z, w.x, w.y, w.z, m.x, m.y, m.z, dt);

    // t_out = ros::Time::now();
    // float dt_tot = (t_out - t_in).toSec() * 1000.0; // In msec.
    // printf("%.6f\n", dt_tot);
    // Publish state.
    publish(imu_msg_raw);
}

tf2::Quaternion ComplementaryFilterROS::hamiltonToTFQuaternion(double q0,
                                                               double q1,
                                                               double q2,
                                                               double q3) const
{
    // ROS uses the Hamilton quaternion convention (q0 is the scalar). However,
    // the ROS quaternion is in the form [x, y, z, w], with w as the scalar.
    return tf2::Quaternion(q1, q2, q3, q0);
}

void ComplementaryFilterROS::publish(ImuMsg::ConstSharedPtr imu_msg_raw)
{
    // Get the orientation:
    double q0, q1, q2, q3;
    filter_.getOrientation(q0, q1, q2, q3);
    tf2::Quaternion q = hamiltonToTFQuaternion(q0, q1, q2, q3);

    // Create and publish fitlered IMU message.
    ImuMsg::SharedPtr imu_msg = std::make_shared<ImuMsg>(*imu_msg_raw);
    imu_msg->orientation.x = q1;
    imu_msg->orientation.y = q2;
    imu_msg->orientation.z = q3;
    imu_msg->orientation.w = q0;

    imu_msg->orientation_covariance[0] = orientation_variance_;
    imu_msg->orientation_covariance[1] = 0.0;
    imu_msg->orientation_covariance[2] = 0.0;
    imu_msg->orientation_covariance[3] = 0.0;
    imu_msg->orientation_covariance[4] = orientation_variance_;
    imu_msg->orientation_covariance[5] = 0.0;
    imu_msg->orientation_covariance[6] = 0.0;
    imu_msg->orientation_covariance[7] = 0.0;
    imu_msg->orientation_covariance[8] = orientation_variance_;

    // Account for biases.
    if (filter_.getDoBiasEstimation())
    {
        imu_msg->angular_velocity.x -= filter_.getAngularVelocityBiasX();
        imu_msg->angular_velocity.y -= filter_.getAngularVelocityBiasY();
        imu_msg->angular_velocity.z -= filter_.getAngularVelocityBiasZ();
    }

    imu_publisher_->publish(*imu_msg);

    if (publish_debug_topics_)
    {
        // Create and publish roll, pitch, yaw angles
        geometry_msgs::msg::Vector3Stamped rpy;
        rpy.header = imu_msg_raw->header;

        tf2::Matrix3x3 M;
        M.setRotation(q);
        M.getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
        rpy_publisher_->publish(rpy);

        // Publish whether we are in the steady state, when doing bias
        // estimation
        if (filter_.getDoBiasEstimation())
        {
            std_msgs::msg::Bool state_msg;
            state_msg.data = filter_.getSteadyState();
            state_publisher_->publish(state_msg);
        }
    }

    if (publish_tf_)
    {
        // Create and publish the ROS tf.
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = imu_msg_raw->header.stamp;
        transform.transform.rotation.x = q1;
        transform.transform.rotation.y = q2;
        transform.transform.rotation.z = q3;
        transform.transform.rotation.w = q0;

        if (reverse_tf_)
        {
            transform.header.frame_id = imu_msg_raw->header.frame_id;
            transform.child_frame_id = fixed_frame_;
            tf_broadcaster_.sendTransform(transform);
        } else
        {
            transform.child_frame_id = imu_msg_raw->header.frame_id;
            transform.header.frame_id = fixed_frame_;
            tf_broadcaster_.sendTransform(transform);
        }
    }
}

}  // namespace imu_tools
