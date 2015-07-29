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

class ImuFilter
{
  public:

    ImuFilter();
    virtual ~ImuFilter();

  private:
    // **** paramaters
    double gain_;     // algorithm gain
    double zeta_;	  // gyro drift bias gain

    // **** state variables
    double q0, q1, q2, q3;  // quaternion
    float w_bx_, w_by_, w_bz_; // 

    // **** member functions

    // Fast inverse square-root
    // See: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Reciprocal_of_the_square_root
    static float invSqrt(float x)
    {
      float xhalf = 0.5f * x;
      union
      {
        float x;
        int i;
      } u;
      u.x = x;
      u.i = 0x5f3759df - (u.i >> 1);
      /* The next line can be repeated any number of times to increase accuracy */
      u.x = u.x * (1.5f - xhalf * u.x * u.x);
      return u.x;
    }

public:
    void setAlgorithmGain(double gain)
    {
        gain_ = gain;
    }

    void setDriftBiasGain(double zeta)
    {
        zeta_ = zeta;
    }

    void getOrientation(double& q0, double& q1, double& q2, double& q3)
    {
        q0 = this->q0;
        q1 = this->q1;
        q2 = this->q2;
        q3 = this->q3;
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
};

#endif // IMU_FILTER_IMU_MADWICK_FILTER_H
