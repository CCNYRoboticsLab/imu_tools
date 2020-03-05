#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

ros::Publisher imu_pub;
void imuCallback(const sensor_msgs::Imu& msg)
{
  sensor_msgs::Imu imu_msg;
  imu_msg = msg;
  imu_msg.orientation_covariance[0] = 0.001;
  imu_msg.orientation_covariance[4] = 0.001;
  imu_msg.orientation_covariance[8] = 0.001;

  imu_pub.publish(imu_msg);
}

int main(int argc, char **argv)
{

ros::init(argc, argv, "imu_add_covariances");
ros::NodeHandle n;
ros::Subscriber imu_sub = n.subscribe("imu/data", 1000, imuCallback);
imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_added_cov_txt", 1000);
ros::spin();

return 0;
}
