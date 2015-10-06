/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

	@section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:
     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
     3. Neither the name of the City College of New York nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL the CCNY ROBOTICS LAB BE LIABLE FOR ANY
	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H

#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "imu_complementary_filter/complementary_filter.h"

namespace imu_tools {

class ComplementaryFilterROS
{
  public:
    ComplementaryFilterROS(const ros::NodeHandle& nh, 
                           const ros::NodeHandle& nh_private);    
    virtual ~ComplementaryFilterROS();

  private:

    // Convenience typedefs
    typedef sensor_msgs::Imu ImuMsg;
    typedef sensor_msgs::MagneticField MagMsg;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, 
        MagMsg> MySyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> 
        SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;    
    typedef message_filters::Subscriber<ImuMsg> ImuSubscriber; 
    typedef message_filters::Subscriber<MagMsg> MagSubscriber;

    // ROS-related variables.
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<MagSubscriber> mag_subscriber_;

    ros::Publisher imu_publisher_;
    ros::Publisher rpy_publisher_;
    ros::Publisher state_publisher_;
    tf::TransformBroadcaster tf_broadcaster_;
         
    // Parameters:
    bool use_mag_;
    bool publish_tf_;
    bool reverse_tf_;
    double constant_dt_;
    bool publish_debug_topics_;
    std::string fixed_frame_;

    // State variables:
    ComplementaryFilter filter_;
    ros::Time time_prev_;
    bool initialized_filter_;

    void initializeParams();
    void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mav_msg);
    void publish(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);

    tf::Quaternion hamiltonToTFQuaternion(
        double q0, double q1, double q2, double q3) const;
};

}  // namespace imu_tools

#endif // IMU_TOOLS_COMPLEMENTARY_FILTER_ROS_H
