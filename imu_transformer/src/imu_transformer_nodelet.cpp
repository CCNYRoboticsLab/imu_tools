#include "imu_transformer/imu_transformer_nodelet.h"
#include "pluginlib/class_list_macros.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace imu_transformer
{

    void ImuTransformerNodelet::onInit(){

      nh_ = ros::NodeHandle(getNodeHandle(), "imu");
      private_nh_ = getPrivateNodeHandle();

      private_nh_.param<std::string>("target_frame", target_frame_, "base_link");

      tf2_.reset(new tf2_ros::Buffer());
      tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));

      imu_sub_.subscribe(nh_, "data_in", 10);
      imu_filter_.reset(new ImuFilter(imu_sub_, *tf2_, target_frame_, 10, nh_));
      imu_sub_.registerCallback(boost::bind(&ImuTransformerNodelet::imuCallback, this, _1));
      imu_filter_->registerFailureCallback(boost::bind(&ImuTransformerNodelet::failureCb, this, _2));

      mag_sub_.subscribe(nh_, "mag_in", 10);
      mag_filter_.reset(new MagFilter(mag_sub_, *tf2_, target_frame_, 10, nh_));
      mag_sub_.registerCallback(boost::bind(&ImuTransformerNodelet::magCallback, this, _1));
      mag_filter_->registerFailureCallback(boost::bind(&ImuTransformerNodelet::failureCb, this, _2));

    }

    void ImuTransformerNodelet::imuCallback(const ImuMsg::ConstPtr &imu_in)
    {
      if(imu_pub_.getTopic().empty()){
        imu_pub_ = nh_.advertise<ImuMsg>("data_out", 10);
      }

      try
      {
        ImuMsg imu_out;
        tf2_->transform(*imu_in, imu_out, target_frame_);
        imu_pub_.publish(imu_out);
      }
      catch (tf2::TransformException ex)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "IMU Transform failure: " << ex.what());
        return;
      }
    }

    void ImuTransformerNodelet::magCallback(const MagMsg::ConstPtr &mag_in)
    {
      if(mag_pub_.getTopic().empty()){
        mag_pub_ = nh_.advertise<MagMsg>("mag_out", 10);
      }

      try
      {
        geometry_msgs::Vector3Stamped mag_out;
        geometry_msgs::TransformStamped transform = tf2_->lookupTransform(target_frame_, mag_in->header.frame_id,
                                                                        mag_in->header.stamp);
        transform.transform.translation.x = 0;
        transform.transform.translation.y = 0;
        transform.transform.translation.z = 0;

        tf2::doTransform(*mag_in, mag_out, transform);
        mag_pub_.publish(mag_out);
      }
      catch (tf2::TransformException ex)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "Mag Transform failure: " << ex.what());
        return;
      }

    }

    void ImuTransformerNodelet::failureCb(tf2_ros::filter_failure_reasons::FilterFailureReason reason)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "Can't transform incoming IMU data to " << target_frame_ <<
          reason);
    }

}

PLUGINLIB_DECLARE_CLASS(imu_transformer, ImuTransformerNodelet, imu_transformer::ImuTransformerNodelet, nodelet::Nodelet);