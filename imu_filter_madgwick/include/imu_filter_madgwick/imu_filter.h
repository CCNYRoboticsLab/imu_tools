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

#ifndef IMU_FILTER_MADWICK_IMU_FILTER_H
#define IMU_FILTER_MADWICK_IMU_FILTER_H

#include <imu_filter_madgwick/world_frame.h>
#include <iostream>
#include <cmath>

class ImuFilter
{
  public:

    ImuFilter();
    virtual ~ImuFilter();

  private:
    // **** paramaters
    double gain_;    // algorithm gain
    double zeta_;    // gyro drift bias gain
    WorldFrame::WorldFrame world_frame_;    // NWU, ENU, NED

    // **** state variables
    double q0, q1, q2, q3;  // quaternion
    float w_bx_, w_by_, w_bz_; //

public:
    void setAlgorithmGain(double gain)
    {
        gain_ = gain;
    }

    void setDriftBiasGain(double zeta)
    {
        zeta_ = zeta;
    }

    void setWorldFrame(WorldFrame::WorldFrame frame)
    {
        world_frame_ = frame;
    }

    void getOrientation(double& q0, double& q1, double& q2, double& q3)
    {
        q0 = this->q0;
        q1 = this->q1;
        q2 = this->q2;
        q3 = this->q3;

        // perform precise normalization of the output, using 1/sqrt()
        // instead of the fast invSqrt() approximation. Without this,
        // TF2 complains that the quaternion is not normalized.
        double recipNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    void setOrientation(double q0, double q1, double q2, double q3)
    {
        this->q0 = q0;
        this->q1 = q1;
        this->q2 = q2;
        this->q3 = q3;

        w_bx_ = 0;
        w_by_ = 0;
        w_bz_ = 0;
    }

    void madgwickAHRSupdate(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float mx, float my, float mz,
                            float dt);

    void madgwickAHRSupdateIMU(float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float dt);

    void getGravity(float& rx, float& ry, float& rz,
                    float gravity = 9.80665);
};

#endif // IMU_FILTER_IMU_MADWICK_FILTER_H
