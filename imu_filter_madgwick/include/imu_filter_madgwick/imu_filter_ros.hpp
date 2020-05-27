#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "imu_filter_madgwick/imu_filter.h"

class ImuFilterMadgwickRos : public rclcpp::Node {
  typedef sensor_msgs::msg::Imu ImuMsg;
  typedef sensor_msgs::msg::MagneticField MagMsg;
  typedef geometry_msgs::msg::Vector3Stamped MagVectorMsg;

  typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber;
  typedef message_filters::Subscriber<MagMsg> MagSubscriber;
  typedef message_filters::Subscriber<MagVectorMsg> MagVectorSubscriber;

  // typedef imu_filter_madgwick::ImuFilterMadgwickConfig   FilterConfig;

public:
  explicit ImuFilterMadgwickRos(const rclcpp::NodeOptions &options);

private:
  std::shared_ptr<ImuSubscriber> imu_subscriber_;
  std::shared_ptr<MagSubscriber> mag_subscriber_;
  std::shared_ptr<Synchronizer> sync_;

  // Adapter to support the use_magnetic_field_msg param.
  std::shared_ptr<MagVectorSubscriber> vector_mag_subscriber_;

  rclcpp::Publisher<MagMsg>::SharedPtr mag_republisher_;
  rclcpp::Publisher<MagVectorMsg>::SharedPtr rpy_filtered_debug_publisher_;
  rclcpp::Publisher<MagVectorMsg>::SharedPtr rpy_raw_debug_publisher_;
  rclcpp::Publisher<ImuMsg>::SharedPtr imu_publisher_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // boost::shared_ptr<FilterConfigServer> config_server_;
  rclcpp::TimerBase::SharedPtr check_topics_timer_;

  // Subscription for parameter change
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  // **** paramaters
  WorldFrame::WorldFrame world_frame_;
  bool use_mag_;
  bool use_magnetic_field_msg_;
  bool stateless_;
  bool publish_tf_;
  bool reverse_tf_;
  std::string fixed_frame_;
  std::string imu_frame_;
  double constant_dt_;
  bool publish_debug_topics_;
  bool remove_gravity_vector_;
  geometry_msgs::msg::Vector3 mag_bias_;
  double orientation_variance_;

  // **** state variables
  std::mutex mutex_;
  bool initialized_;
  rclcpp::Time last_time_;

  // **** filter implementation
  ImuFilter filter_;

  // **** member functions
  void imuMagCallback(const ImuMsg::SharedPtr imu_msg_raw, const MagMsg::SharedPtr mav_msg);

  void imuCallback(const ImuMsg::SharedPtr imu_msg_raw);

  void imuMagVectorCallback(const MagVectorMsg::SharedPtr mag_vector_msg);

  void publishFilteredMsg(const ImuMsg::SharedPtr imu_msg_raw);
  void publishTransform(const ImuMsg::SharedPtr imu_msg_raw);

  void publishRawMsg(const rclcpp::Time &t, float roll, float pitch, float yaw);

  void reconfigCallback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  void checkTopicsTimerCallback();
};