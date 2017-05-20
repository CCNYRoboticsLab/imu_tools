//Remove this header when https://github.com/ros/geometry_experimental/pull/78 is released

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef TF2_SENSOR_MSGS_IMU_H
#define TF2_SENSOR_MSGS_IMU_H

#include <tf2/convert.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace tf2
{

/**********/
/** IMU  **/
/**********/

/**
* method to extract timestamp from object
*/
  template <>
  inline
  const ros::Time& getTimestamp(const sensor_msgs::Imu& p) {return p.header.stamp;}

/**
* method to extract frame id from object
*/
  template <>
  inline
  const std::string& getFrameId(const sensor_msgs::Imu &p) {return p.header.frame_id;}


/**
* Transforms a covariance array from one frame to another
*/
  inline
  void transformCovariance(const boost::array<double, 9>& in, boost::array<double, 9>& out, Eigen::Quaternion<double> r){

    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_in(in.data());
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > cov_out(out.data());
    cov_out = r * cov_in * r.inverse();

  }

/**
* Transforms sensor_msgs::Imu data from one frame to another
*/
  template <>
  inline
  void doTransform(const sensor_msgs::Imu &imu_in, sensor_msgs::Imu &imu_out, const geometry_msgs::TransformStamped& t_in)
  {

    imu_out.header = t_in.header;

    // Discard translation, only use orientation for IMU transform
    Eigen::Quaternion<double> r(
        t_in.transform.rotation.w, t_in.transform.rotation.x, t_in.transform.rotation.y, t_in.transform.rotation.z);
    Eigen::Transform<double,3,Eigen::Affine> t(r);

    Eigen::Vector3d vel = t * Eigen::Vector3d(
        imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);

    imu_out.angular_velocity.x = vel.x();
    imu_out.angular_velocity.y = vel.y();
    imu_out.angular_velocity.z = vel.z();

    transformCovariance(imu_in.angular_velocity_covariance, imu_out.angular_velocity_covariance, r);

    Eigen::Vector3d accel = t * Eigen::Vector3d(
        imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);


    imu_out.linear_acceleration.x = accel.x();
    imu_out.linear_acceleration.y = accel.y();
    imu_out.linear_acceleration.z = accel.z();

    transformCovariance(imu_in.linear_acceleration_covariance, imu_out.linear_acceleration_covariance, r);

    Eigen::Quaternion<double> orientation = r * Eigen::Quaternion<double>(
        imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z) * r.inverse();

    imu_out.orientation.w = orientation.w();
    imu_out.orientation.x = orientation.x();
    imu_out.orientation.y = orientation.y();
    imu_out.orientation.z = orientation.z();

    transformCovariance(imu_in.orientation_covariance, imu_out.orientation_covariance, r);

  }

  inline
  sensor_msgs::Imu toMsg(const sensor_msgs::Imu &in)
  {
    return in;
  }

  inline
  void fromMsg(const sensor_msgs::Imu &msg, sensor_msgs::Imu &out)
  {
    out = msg;
  }

} // namespace

#endif // TF2_SENSOR_MSGS_IMU_H
