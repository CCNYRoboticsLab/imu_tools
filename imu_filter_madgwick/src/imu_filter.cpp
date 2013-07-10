/*
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  http://robotics.ccny.cuny.edu
 *
 *  Based on implementation of Madgwick's IMU and AHRS algorithms.
 *  http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "imu_filter_madgwick/imu_filter.h"

ImuFilter::ImuFilter(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  q0(1.0), q1(0.0), q2(0.0), q3(0.0)
{
  ROS_INFO ("Starting ImuFilter");

  // **** get paramters

  if (!nh_private_.getParam ("gain", gain_))
   gain_ = 0.1;
  if (!nh_private_.getParam ("zeta", zeta_))
   zeta_ = 0;
  if (!nh_private_.getParam ("use_mag", use_mag_))
   use_mag_ = true;
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
   publish_tf_ = true;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
   fixed_frame_ = "odom";
  if (!nh_private_.getParam ("constant_dt", constant_dt_))
    constant_dt_ = 0.0;

  // check for illegal constant_dt values
  if (constant_dt_ < 0.0)
  {
    ROS_FATAL("constant_dt parameter is %f, must be >= 0.0. Setting to 0.0", constant_dt_);
    constant_dt_ = 0.0;
  }
  
  // if constant_dt_ is 0.0 (default), use IMU timestamp to determine dt
  // otherwise, it will be constant
  if (constant_dt_ == 0.0)
    ROS_INFO("Using dt computed from message headers");
  else
    ROS_INFO("Using constant dt of %f sec", constant_dt_);

  // **** register dynamic reconfigure
  
  FilterConfigServer::CallbackType f = boost::bind(&ImuFilter::reconfigCallback, this, _1, _2);
  config_server_.setCallback(f);
  
  // **** register publishers

  imu_publisher_ = nh_.advertise<sensor_msgs::Imu>(
    "imu/data", 5);

  // **** register subscribers

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;

  imu_subscriber_.reset(new ImuSubscriber(
    nh_, "imu/data_raw", queue_size));

  if (use_mag_)
  {
    mag_subscriber_.reset(new MagSubscriber(
      nh_, "imu/mag", queue_size));

    sync_.reset(new Synchronizer(
      SyncPolicy(queue_size), *imu_subscriber_, *mag_subscriber_));
    sync_->registerCallback(boost::bind(&ImuFilter::imuMagCallback, this, _1, _2));
  }
  else
  {
    imu_subscriber_->registerCallback(&ImuFilter::imuCallback, this);
  }
}

ImuFilter::~ImuFilter()
{
  ROS_INFO ("Destroying ImuFilter");
}

void ImuFilter::imuCallback(const ImuMsg::ConstPtr& imu_msg_raw)
{
  boost::mutex::scoped_lock(mutex_);

  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  
  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  if (!initialized_)
  {
    // initialize roll/pitch orientation from acc. vector    
    double sign = copysignf(1.0, lin_acc.z);
    double roll = atan2(lin_acc.y, sign * sqrt(lin_acc.x*lin_acc.x + lin_acc.z*lin_acc.z));
    double pitch = -atan2(lin_acc.x, sqrt(lin_acc.y*lin_acc.y + lin_acc.z*lin_acc.z));
    double yaw = 0.0;
                        
    tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    
    q1 = init_q.getX();
    q2 = init_q.getY();
    q3 = init_q.getZ();
    q0 = init_q.getW();
    
    // initialize time
    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - last_time_).toSec();

  last_time_ = time;
  
  madgwickAHRSupdateIMU(
    ang_vel.x, ang_vel.y, ang_vel.z,
    lin_acc.x, lin_acc.y, lin_acc.z,
    dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);
}

void ImuFilter::imuMagCallback(
  const ImuMsg::ConstPtr& imu_msg_raw,
  const MagMsg::ConstPtr& mag_msg)
{
  boost::mutex::scoped_lock(mutex_);
  
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
  const geometry_msgs::Vector3& mag_fld = mag_msg->vector;
  
  ros::Time time = imu_msg_raw->header.stamp;
  imu_frame_ = imu_msg_raw->header.frame_id;

  if (!initialized_)
  {
    // initialize roll/pitch orientation from acc. vector    
    double sign = copysignf(1.0, lin_acc.z);
    double roll = atan2(lin_acc.y, sign * sqrt(lin_acc.x*lin_acc.x + lin_acc.z*lin_acc.z));
    double pitch = -atan2(lin_acc.x, sqrt(lin_acc.y*lin_acc.y + lin_acc.z*lin_acc.z));
    double yaw = 0.0; // TODO: initialize from magnetic raeding?
                        
    tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    
    q1 = init_q.getX();
    q2 = init_q.getY();
    q3 = init_q.getZ();
    q0 = init_q.getW();
    
    w_bx_ = 0;
    w_by_ = 0;
    w_bz_ = 0;
    
    last_time_ = time;
    initialized_ = true;
  }

  // determine dt: either constant, or from IMU timestamp
  float dt;
  if (constant_dt_ > 0.0)
    dt = constant_dt_;
  else
    dt = (time - last_time_).toSec();
  
  last_time_ = time;

  madgwickAHRSupdate(
    ang_vel.x, ang_vel.y, ang_vel.z,
    lin_acc.x, lin_acc.y, lin_acc.z,
    mag_fld.x, mag_fld.y, mag_fld.z,
    dt);

  publishFilteredMsg(imu_msg_raw);
  if (publish_tf_)
    publishTransform(imu_msg_raw);
}

void ImuFilter::publishTransform(const ImuMsg::ConstPtr& imu_msg_raw)
{
  tf::Quaternion q(q1, q2, q3, q0);
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
  transform.setRotation( q );
  tf_broadcaster_.sendTransform( tf::StampedTransform( transform,
                   imu_msg_raw->header.stamp,
                   fixed_frame_,
                   imu_frame_ ) );

}

void ImuFilter::publishFilteredMsg(const ImuMsg::ConstPtr& imu_msg_raw)
{
  // create orientation quaternion
  // q0 is the angle, q1, q2, q3 are the axes  
  tf::Quaternion q(q1, q2, q3, q0);   

  // create and publish fitlered IMU message
  boost::shared_ptr<ImuMsg> imu_msg = 
    boost::make_shared<ImuMsg>(*imu_msg_raw);

  imu_msg->header.frame_id = fixed_frame_;
  tf::quaternionTFToMsg(q, imu_msg->orientation);  
  imu_publisher_.publish(imu_msg);
}

void ImuFilter::madgwickAHRSupdate(
  float gx, float gy, float gz, 
  float ax, float ay, float az, 
  float mx, float my, float mz, 
  float dt)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float _w_err_x, _w_err_y, _w_err_z;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
  if(std::isnan(mx) || std::isnan(my) || std::isnan(mz))
  {
		madgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az, dt);
		return;
	}

	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
  {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// compute gyro drift bias
		_w_err_x = _2q0 * s1 - _2q1 * s0 - _2q2 * s3 + _2q3 * s2;
		_w_err_y = _2q0 * s2 + _2q1 * s3 - _2q2 * s0 - _2q3 * s1;
		_w_err_z = _2q0 * s3 - _2q1 * s2 + _2q2 * s1 - _2q3 * s0;
		
		w_bx_ += _w_err_x * dt * zeta_;
		w_by_ += _w_err_y * dt * zeta_;
		w_bz_ += _w_err_z * dt * zeta_;
		
		gx -= w_bx_;
		gy -= w_by_;
		gz -= w_bz_;
		
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

		// Apply feedback step
		qDot1 -= gain_ * s0;
		qDot2 -= gain_ * s1;
		qDot3 -= gain_ * s2;
		qDot4 -= gain_ * s3;
	} else{
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


void ImuFilter::madgwickAHRSupdateIMU(
  float gx, float gy, float gz, 
  float ax, float ay, float az,
  float dt) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
  {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= gain_ * s0;
		qDot2 -= gain_ * s1;
		qDot3 -= gain_ * s2;
		qDot4 -= gain_ * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void ImuFilter::reconfigCallback(FilterConfig& config, uint32_t level)
{
  boost::mutex::scoped_lock(mutex_);
  gain_ = config.gain;
  zeta_ = config.zeta;
  ROS_INFO("Imu filter gain set to %f", gain_);
  ROS_INFO("Gyro drift bias set to %f", zeta_);
}


