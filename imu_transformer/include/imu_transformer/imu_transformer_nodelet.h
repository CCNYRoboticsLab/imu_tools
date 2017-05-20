#ifndef IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET
#define IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET

#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nodelet/nodelet.h"
#include "message_filters/subscriber.h"

#include <string>

namespace imu_transformer
{
  typedef sensor_msgs::Imu              ImuMsg;
  typedef geometry_msgs::Vector3Stamped MagMsg;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
  typedef message_filters::Subscriber<MagMsg> MagSubscriber;
  typedef tf2_ros::MessageFilter<ImuMsg> ImuFilter;
  typedef tf2_ros::MessageFilter<MagMsg> MagFilter;

  class ImuTransformerNodelet : public nodelet::Nodelet
  {

  public:
    ImuTransformerNodelet() {};

  private:

    std::string target_frame_;

    ros::NodeHandle nh_, private_nh_;
    boost::shared_ptr<tf2_ros::Buffer> tf2_;
    boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    ImuSubscriber imu_sub_;
    MagSubscriber mag_sub_;

    boost::shared_ptr<ImuFilter> imu_filter_;
    boost::shared_ptr<MagFilter> mag_filter_;

    ros::Publisher imu_pub_, mag_pub_;

    virtual void onInit();
    void imuCallback(const ImuMsg::ConstPtr &imu_in);
    void magCallback(const MagMsg::ConstPtr &mag_in);
    void failureCb(tf2_ros::filter_failure_reasons::FilterFailureReason reason);

  };

}  // namespace imu_transformer

#endif  // IMU_TRANSFORMER_IMU_TRANSFORMER_NODELET