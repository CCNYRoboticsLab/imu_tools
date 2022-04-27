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
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "imu_filter_madgwick/imu_filter.h"
#include "imu_filter_madgwick/base_node.hpp"
#include "imu_filter_madgwick/visibility_control.h"

class ImuFilterMadgwickRos : public imu_filter::BaseNode
{
    typedef sensor_msgs::msg::Imu ImuMsg;
    typedef sensor_msgs::msg::MagneticField MagMsg;
    typedef geometry_msgs::msg::Vector3Stamped RpyVectorMsg;

    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>
        SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

  public:
    IMU_FILTER_MADGWICK_CPP_PUBLIC
    explicit ImuFilterMadgwickRos(const rclcpp::NodeOptions& options);

    // Callbacks are public so they can be called when used as a library
    void imuCallback(ImuMsg::ConstSharedPtr imu_msg_raw);
    void imuMagCallback(ImuMsg::ConstSharedPtr imu_msg_raw,
                        MagMsg::ConstSharedPtr mag_msg);

  private:
    std::shared_ptr<ImuSubscriber> imu_subscriber_;
    std::shared_ptr<MagSubscriber> mag_subscriber_;
    std::shared_ptr<Synchronizer> sync_;

    rclcpp::Publisher<RpyVectorMsg>::SharedPtr rpy_filtered_debug_publisher_;
    rclcpp::Publisher<RpyVectorMsg>::SharedPtr rpy_raw_debug_publisher_;
    rclcpp::Publisher<ImuMsg>::SharedPtr imu_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr check_topics_timer_;

    // Subscription for parameter change
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
        parameter_event_sub_;

    // **** paramaters
    WorldFrame::WorldFrame world_frame_;
    bool use_mag_{};
    bool stateless_{};
    bool publish_tf_{};
    bool reverse_tf_{};
    std::string fixed_frame_;
    std::string imu_frame_;
    double constant_dt_;
    bool publish_debug_topics_{};
    bool remove_gravity_vector_{};
    geometry_msgs::msg::Vector3 mag_bias_;
    double orientation_variance_;
    double yaw_offset_total_;

    // **** state variables
    std::mutex mutex_;
    bool initialized_;
    rclcpp::Time last_time_;
    tf2::Quaternion yaw_offsets_;

    // **** filter implementation
    ImuFilter filter_;

    // **** member functions
    void publishFilteredMsg(ImuMsg::ConstSharedPtr imu_msg_raw);
    void publishTransform(ImuMsg::ConstSharedPtr imu_msg_raw);

    void publishRawMsg(const rclcpp::Time& t, float roll, float pitch,
                       float yaw);

    void reconfigCallback(rcl_interfaces::msg::ParameterEvent::SharedPtr event);
    void checkTopicsTimerCallback();

    void applyYawOffset(double& q0, double& q1, double& q2, double& q3);
};
