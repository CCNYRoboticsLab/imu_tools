#ifndef IMU_FILTER_IMU_FILTER_H
#define IMU_FILTER_IMU_FILTER_H

#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class ImuFilter
{
  typedef sensor_msgs::Imu              ImuMsg;
  typedef geometry_msgs::Vector3Stamped MagMsg;

  typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef message_filters::Subscriber<ImuMsg> ImuSubscriber; 
  typedef message_filters::Subscriber<MagMsg> MagSubscriber;

  public:

    ImuFilter(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~ImuFilter();

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    boost::shared_ptr<Synchronizer> sync_;
    boost::shared_ptr<ImuSubscriber> imu_subscriber_;
    boost::shared_ptr<MagSubscriber> mag_subscriber_;

    ros::Publisher imu_publisher_;
  
    // **** paramaters

    double gain_;				// algorithm gain
    bool use_mag_;
    std::string fixed_frame_;

    // **** state variables
  
    bool initialized_;
    double q0, q1, q2, q3;	// quaternion
    ros::Time last_time_;

    // **** member functions

    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mav_msg);

    void imuCallback(const ImuMsg::ConstPtr& imu_msg_raw);

    void publishFilteredMsg(const ImuMsg::ConstPtr& imu_msg_raw);

    void madgwickAHRSupdate(float gx, float gy, float gz, 
                            float ax, float ay, float az, 
                            float mx, float my, float mz,
                            float dt);

    void madgwickAHRSupdateIMU(float gx, float gy, float gz, 
                               float ax, float ay, float az,
                               float dt);

    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
    static float invSqrt(float x) 
    {
	    float halfx = 0.5f * x;
	    float y = x;
	    long i = *(long*)&y;
	    i = 0x5f3759df - (i>>1);
	    y = *(float*)&i;
	    y = y * (1.5f - (halfx * y * y));
	    return y;
    }
};

#endif // IMU_FILTER_IMU_FILTER_H
