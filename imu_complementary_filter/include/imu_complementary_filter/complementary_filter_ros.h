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

#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H

#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

#include "imu_complementary_filter/complementary_filter.h"

namespace imu_tools {

class ComplementaryFilterROS : public rclcpp::Node
{
  public:
    ComplementaryFilterROS();
    ~ComplementaryFilterROS() override;

  private:
    // Convenience typedefs
    typedef sensor_msgs::msg::Imu ImuMsg;
    typedef sensor_msgs::msg::MagneticField MagMsg;
    typedef geometry_msgs::msg::Vector3Stamped RpyVectorMsg;
    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg>
        SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

    // ROS-related variables.
    std::shared_ptr<ImuSubscriber> imu_subscriber_;
    std::shared_ptr<MagSubscriber> mag_subscriber_;
    std::shared_ptr<Synchronizer> sync_;

    rclcpp::Publisher<ImuMsg>::SharedPtr imu_publisher_;
    rclcpp::Publisher<RpyVectorMsg>::SharedPtr rpy_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // Parameters:
    bool use_mag_{};
    bool publish_tf_{};
    bool reverse_tf_{};
    double constant_dt_{};
    bool publish_debug_topics_{};
    std::string fixed_frame_;
    double orientation_variance_{};

    // State variables:
    ComplementaryFilter filter_;
    rclcpp::Time time_prev_;
    bool initialized_filter_;

    void initializeParams();
    void imuCallback(ImuMsg::ConstSharedPtr imu_msg_raw);
    void imuMagCallback(ImuMsg::ConstSharedPtr imu_msg_raw,
                        MagMsg::ConstSharedPtr mav_msg);
    void publish(ImuMsg::ConstSharedPtr imu_msg_raw);

    tf2::Quaternion hamiltonToTFQuaternion(double q0, double q1, double q2,
                                           double q3) const;
};

}  // namespace imu_tools

#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
