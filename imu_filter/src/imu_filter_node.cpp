#include "imu_filter/imu_filter.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ImuFilter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ImuFilter imu_filter(nh, nh_private);
  ros::spin();
  return 0;
}
