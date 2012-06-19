#include "imu_filter/imu_filter_nodelet.h"

PLUGINLIB_DECLARE_CLASS(imu_filter, ImuFilterNodelet, ImuFilterNodelet, nodelet::Nodelet);

void ImuFilterNodelet::onInit()
{
  NODELET_INFO("Initializing IMU Filter Nodelet");
  
  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh         = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  filter_ = new ImuFilter(nh, nh_private);
}
