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
    : BaseNode("imu_filter_madgwick", options),
      tf_broadcaster_(this),
      initialized_(false)
{
    RCLCPP_INFO(get_logger(), "Starting ImuFilter");

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

    double yaw_offset = 0.0;
    declare_parameter("yaw_offset", 0.0);
    get_parameter("yaw_offset", yaw_offset);
    double declination = 0.0;
    declare_parameter("declination", 0.0);
    get_parameter("declination", declination);

    // create yaw offset quaternion
    yaw_offset_total_ = yaw_offset - declination;
    yaw_offsets_.setRPY(
        0, 0,
        yaw_offset_total_);  // Create this quaternion for yaw offset (radians)

    std::string world_frame;
    declare_parameter("world_frame", "enu");
    get_parameter("world_frame", world_frame);
    if (world_frame == "ned")
    {
        world_frame_ = WorldFrame::NED;
    } else if (world_frame == "nwu")
    {
        world_frame_ = WorldFrame::NWU;
    } else if (world_frame == "enu")
    {
        world_frame_ = WorldFrame::ENU;
    } else
    {
        RCLCPP_ERROR(get_logger(),
                     "The parameter world_frame was set to invalid value '%s'.",
                     world_frame.c_str());
        RCLCPP_ERROR(
            get_logger(),
            "Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
        world_frame_ = WorldFrame::ENU;
    }
    filter_.setWorldFrame(world_frame_);

    // check for illegal constant_dt values
    if (constant_dt_ < 0.0)
    {
        RCLCPP_ERROR(
            get_logger(),
            "constant_dt parameter is %f, must be >= 0.0. Setting to 0.0",
            constant_dt_);
        constant_dt_ = 0.0;
    }

    // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
    // otherwise, it will be constant
    if (constant_dt_ == 0.0)
    {
        RCLCPP_INFO(get_logger(), "Using dt computed from message headers");
    } else
    {
        RCLCPP_INFO(get_logger(), "Using constant dt of %f sec", constant_dt_);
    }

    if (remove_gravity_vector_)
    {
        RCLCPP_INFO(get_logger(),
                    "The gravity vector will be removed from the acceleration");
    } else
    {
        RCLCPP_INFO(get_logger(),
                    "The gravity vector is kept in the IMU message.");
    }

    // **** define reconfigurable parameters.
    double gain;
    floating_point_range float_range = {0.0, 1.0, 0};
    add_parameter(
        "gain", rclcpp::ParameterValue(0.1), float_range,
        "Gain of the filter. Higher values lead to faster convergence but"
        "more noise. Lower values lead to slower convergence but smoother "
        "signal.");
    double zeta;
    float_range = {-1.0, 1.0, 0};
    add_parameter("zeta", rclcpp::ParameterValue(0.0), float_range,
                  "Gyro drift gain (approx. rad/s).");
    double mag_bias_x;
    float_range = {-10.0, 10.0, 0};
    add_parameter("mag_bias_x", rclcpp::ParameterValue(0.0), float_range,
                  "Magnetometer bias (hard iron correction), x component.");
    double mag_bias_y;
    add_parameter("mag_bias_y", rclcpp::ParameterValue(0.0), float_range,
                  "Magnetometer bias (hard iron correction), y component.");
    double mag_bias_z;
    add_parameter("mag_bias_z", rclcpp::ParameterValue(0.0), float_range,
                  "Magnetometer bias (hard iron correction), z component.");
    double orientation_stddev;
    float_range = {0.0, 1.0, 0};
    add_parameter("orientation_stddev", rclcpp::ParameterValue(0.0),
                  float_range,
                  "Standard deviation of the orientation estimate.");

    // **** get initial values of reconfigurable parameters.
    get_parameter("gain", gain);
    get_parameter("zeta", zeta);
    get_parameter("mag_bias_x", mag_bias_x);
    get_parameter("mag_bias_y", mag_bias_y);
    get_parameter("mag_bias_z", mag_bias_z);
    get_parameter("orientation_stddev", orientation_stddev);

    filter_.setAlgorithmGain(gain);
    filter_.setDriftBiasGain(zeta);
    RCLCPP_INFO(get_logger(), "Imu filter gain set to %f", gain);
    RCLCPP_INFO(get_logger(), "Gyro drift bias set to %f", zeta);
    mag_bias_.x = mag_bias_x;
    mag_bias_.y = mag_bias_y;
    mag_bias_.z = mag_bias_z;
    orientation_variance_ = orientation_stddev * orientation_stddev;
    RCLCPP_INFO(get_logger(), "Magnetometer bias values: %f %f %f", mag_bias_.x,
                mag_bias_.y, mag_bias_.z);

    // **** register dynamic reconfigure
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(), this->get_node_topics_interface(),
        this->get_node_graph_interface(), this->get_node_services_interface());

    parameter_event_sub_ = parameters_client_->on_parameter_event(
        std::bind(&ImuFilterMadgwickRos::reconfigCallback, this, _1));

    // **** register publishers
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 5);
    if (publish_debug_topics_)
    {
        rpy_filtered_debug_publisher_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "imu/rpy/filtered", 5);

        rpy_raw_debug_publisher_ =
            create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy/raw",
                                                                 5);
    }

    // **** register subscribers
    // Synchronize inputs. Topic subscriptions happen on demand in the
    // connection callback.
    const int queue_size = 5;
    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    imu_subscriber_.reset(new ImuSubscriber(this, "imu/data_raw", qos));

    if (use_mag_)
    {
        mag_subscriber_.reset(new MagSubscriber(this, "imu/mag", qos));

        sync_.reset(new Synchronizer(SyncPolicy(queue_size), *imu_subscriber_,
                                     *mag_subscriber_));
        sync_->registerCallback(&ImuFilterMadgwickRos::imuMagCallback, this);
    } else
    {
        imu_subscriber_->registerCallback(&ImuFilterMadgwickRos::imuCallback,
                                          this);
    }

    check_topics_timer_ = create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&ImuFilterMadgwickRos::checkTopicsTimerCallback, this));
}

void ImuFilterMadgwickRos::imuCallback(ImuMsg::ConstSharedPtr imu_msg_raw)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const geometry_msgs::msg::Vector3 &ang_vel = imu_msg_raw->angular_velocity;
    const geometry_msgs::msg::Vector3 &lin_acc =
        imu_msg_raw->linear_acceleration;

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);  // for throttle logger message

    rclcpp::Time time = imu_msg_raw->header.stamp;
    imu_frame_ = imu_msg_raw->header.frame_id;

    if (!initialized_ || stateless_)
    {
        geometry_msgs::msg::Quaternion init_q;
        if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc,
                                                      init_q))
        {
            RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 5.0,
                                 "The IMU seems to be in free fall, cannot "
                                 "determine gravity direction!");
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
        if (0 == time.nanoseconds())
        {
            RCLCPP_WARN_STREAM_THROTTLE(
                get_logger(), steady_clock, 5.0,
                "The IMU message time stamp is zero, and the parameter "
                "constant_dt is not set!"
                    << " The filter will not update the orientation.");
        }
    }

    last_time_ = time;

    if (!stateless_)
        filter_.madgwickAHRSupdateIMU(ang_vel.x, ang_vel.y, ang_vel.z,
                                      lin_acc.x, lin_acc.y, lin_acc.z, dt);

    publishFilteredMsg(imu_msg_raw);
    if (publish_tf_) publishTransform(imu_msg_raw);
}

void ImuFilterMadgwickRos::imuMagCallback(ImuMsg::ConstSharedPtr imu_msg_raw,
                                          MagMsg::ConstSharedPtr mag_msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    const geometry_msgs::msg::Vector3 &ang_vel = imu_msg_raw->angular_velocity;
    const geometry_msgs::msg::Vector3 &lin_acc =
        imu_msg_raw->linear_acceleration;
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

    rclcpp::Clock steady_clock(RCL_STEADY_TIME);  // for throttle logger message

    if (!initialized_ || stateless_)
    {
        // wait for mag message without NaN / inf
        if (!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) ||
            !std::isfinite(mag_fld.z))
        {
            return;
        }

        geometry_msgs::msg::Quaternion init_q;
        if (!StatelessOrientation::computeOrientation(world_frame_, lin_acc,
                                                      mag_compensated, init_q))
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), steady_clock, 5.0,
                "The IMU seems to be in free fall or close to magnetic north "
                "pole, cannot determine gravity direction!");
            return;
        }
        filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
    }

    if (!initialized_)
    {
        RCLCPP_INFO(get_logger(),
                    "First pair of IMU and magnetometer messages received.");
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
        if (0 == time.nanoseconds())
        {
            RCLCPP_WARN_STREAM_THROTTLE(
                get_logger(), steady_clock, 5.0,
                "The IMU message time stamp is zero, and the parameter "
                "constant_dt is not set!"
                    << " The filter will not update the orientation.");
        }
    }

    last_time_ = time;

    if (!stateless_)
    {
        filter_.madgwickAHRSupdate(ang_vel.x, ang_vel.y, ang_vel.z, lin_acc.x,
                                   lin_acc.y, lin_acc.z, mag_compensated.x,
                                   mag_compensated.y, mag_compensated.z, dt);
    }

    publishFilteredMsg(imu_msg_raw);
    if (publish_tf_)
    {
        publishTransform(imu_msg_raw);
    }

    if (publish_debug_topics_)
    {
        geometry_msgs::msg::Quaternion orientation;
        if (StatelessOrientation::computeOrientation(
                world_frame_, lin_acc, mag_compensated, orientation))
        {
            tf2::Matrix3x3(tf2::Quaternion(orientation.x, orientation.y,
                                           orientation.z, orientation.w))
                .getRPY(roll, pitch, yaw, 0);
            publishRawMsg(time, roll, pitch, yaw);
        }
    }
}

void ImuFilterMadgwickRos::publishTransform(ImuMsg::ConstSharedPtr imu_msg_raw)
{
    double q0, q1, q2, q3;
    filter_.getOrientation(q0, q1, q2, q3);
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
    } else
    {
        transform.header.frame_id = fixed_frame_;
        transform.child_frame_id = imu_frame_;
        transform.transform.rotation.w = q0;
        transform.transform.rotation.x = q1;
        transform.transform.rotation.y = q2;
        transform.transform.rotation.z = q3;
    }
    tf_broadcaster_.sendTransform(transform);
}

/**
 * @brief Applies yaw offset quaternion (yaw offset - declination) to the
 *orientation quaternion. Alters the quaternion if there is a yaw offset.
 * @param q0 quaternion x component
 * @param q1 quaternion y component
 * @param q2 quaternion z component
 * @param q3 quaternion w component
 **/
void ImuFilterMadgwickRos::applyYawOffset(double &q0, double &q1, double &q2,
                                          double &q3)
{
    if (yaw_offset_total_ != 0.0)
    {
        tf2::Quaternion offset_orientation =
            yaw_offsets_ * tf2::Quaternion(q1, q2, q3, q0);
        offset_orientation.normalize();
        q1 = offset_orientation.x();
        q2 = offset_orientation.y();
        q3 = offset_orientation.z();
        q0 = offset_orientation.w();
    }
}

void ImuFilterMadgwickRos::publishFilteredMsg(
    ImuMsg::ConstSharedPtr imu_msg_raw)
{
    double q0, q1, q2, q3;
    filter_.getOrientation(q0, q1, q2, q3);
    // apply yaw offsets
    applyYawOffset(q0, q1, q2, q3);

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

    if (remove_gravity_vector_)
    {
        float gx, gy, gz;
        filter_.getGravity(gx, gy, gz);
        imu_msg.linear_acceleration.x -= gx;
        imu_msg.linear_acceleration.y -= gy;
        imu_msg.linear_acceleration.z -= gz;
    }

    imu_publisher_->publish(imu_msg);

    if (publish_debug_topics_)
    {
        geometry_msgs::msg::Vector3Stamped rpy;
        tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0))
            .getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);

        rpy.header = imu_msg_raw->header;
        rpy_filtered_debug_publisher_->publish(rpy);
    }
}

void ImuFilterMadgwickRos::publishRawMsg(const rclcpp::Time &t, float roll,
                                         float pitch, float yaw)
{
    geometry_msgs::msg::Vector3Stamped rpy;
    rpy.vector.x = roll;
    rpy.vector.y = pitch;
    rpy.vector.z = yaw;
    rpy.header.stamp = t;
    rpy.header.frame_id = imu_frame_;
    rpy_raw_debug_publisher_->publish(rpy);
}

void ImuFilterMadgwickRos::reconfigCallback(
    const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
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
            RCLCPP_INFO(get_logger(), "Parameter %s set to %f", name.c_str(),
                        value.double_value);
            if (name == "gain")
            {
                gain = value.double_value;
                filter_.setAlgorithmGain(gain);
            } else if (name == "zeta")
            {
                zeta = value.double_value;
                filter_.setDriftBiasGain(zeta);
            } else if (name == "mag_bias_x")
            {
                mag_bias_.x = value.double_value;
            } else if (name == "mag_bias_y")
            {
                mag_bias_.y = value.double_value;
            } else if (name == "mag_bias_z")
            {
                mag_bias_.z = value.double_value;
            } else if (name == "orientation_stddev")
            {
                double orientation_stddev = value.double_value;
                orientation_variance_ = orientation_stddev * orientation_stddev;
            }
        }
    }
}

void ImuFilterMadgwickRos::checkTopicsTimerCallback()
{
    if (use_mag_)
        RCLCPP_WARN_STREAM(
            get_logger(),
            "Still waiting for data on topics /imu/data_raw and /imu/mag...");
    else
        RCLCPP_WARN_STREAM(get_logger(),
                           "Still waiting for data on topic /imu/data_raw...");
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ImuFilterMadgwickRos)
