#include "imu_complementary_filter/complementary_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

void ComplementaryFilterNodelet::onInit()
{
    NODELET_INFO("Initializing Complementary Filter Nodelet");

    ros::NodeHandle nh = getMTNodeHandle();
    ros::NodeHandle nh_private = getMTPrivateNodeHandle();

    filter_.reset(new imu_tools::ComplementaryFilterROS(nh, nh_private));
}

PLUGINLIB_EXPORT_CLASS(ComplementaryFilterNodelet, nodelet::Nodelet)
