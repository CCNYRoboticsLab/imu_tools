/*
  @author Roberto G. Valenti <robertogl.valenti@gmail.com>

  @section LICENSE
  Copyright (c) 2015, City University of New York
  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived from
        this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef IMU_TOOLS_COMPLEMENTARY_FILTER_H
#define IMU_TOOLS_COMPLEMENTARY_FILTER_H

namespace imu_tools {

class ComplementaryFilter
{
  public:
    ComplementaryFilter();
    virtual ~ComplementaryFilter();

    bool setGainAcc(double gain);
    bool setGainMag(double gain);
    double getGainAcc() const;
    double getGainMag() const;

    bool setBiasAlpha(double bias_alpha);
    double getBiasAlpha() const;

    // When the filter is in the steady state, bias estimation will occur (if
    // the parameter is enabled).
    bool getSteadyState() const;

    void setDoBiasEstimation(bool do_bias_estimation);
    bool getDoBiasEstimation() const;

    void setDoAdaptiveGain(bool do_adaptive_gain);
    bool getDoAdaptiveGain() const;

    double getAngularVelocityBiasX() const;
    double getAngularVelocityBiasY() const;
    double getAngularVelocityBiasZ() const;

    // Set the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void setOrientation(double q0, double q1, double q2, double q3);

    // Get the orientation, as a Hamilton Quaternion, of the body frame wrt the
    // fixed frame.
    void getOrientation(double& q0, double& q1, double& q2, double& q3) const;

    // Update from accelerometer and gyroscope data.
    // [ax, ay, az]: Normalized gravity vector.
    // [wx, wy, wz]: Angular veloctiy, in rad / s.
    // dt: time delta, in seconds.
    void update(double ax, double ay, double az, double wx, double wy,
                double wz, double dt);

    // Update from accelerometer, gyroscope, and magnetometer data.
    // [ax, ay, az]: Normalized gravity vector.
    // [wx, wy, wz]: Angular veloctiy, in rad / s.
    // [mx, my, mz]: Magnetic field, units irrelevant.
    // dt: time delta, in seconds.
    void update(double ax, double ay, double az, double wx, double wy,
                double wz, double mx, double my, double mz, double dt);

    // Reset the filter to the initial state.
    void reset();

  private:
    static const double kGravity;
    static const double gamma_;
    // Bias estimation steady state thresholds
    static const double kAngularVelocityThreshold;
    static const double kAccelerationThreshold;
    static const double kDeltaAngularVelocityThreshold;

    // Gain parameter for the complementary filter, belongs in [0, 1].
    double gain_acc_;
    double gain_mag_;

    // Bias estimation gain parameter, belongs in [0, 1].
    double bias_alpha_;

    // Parameter whether to do bias estimation or not.
    bool do_bias_estimation_;

    // Parameter whether to do adaptive gain or not.
    bool do_adaptive_gain_;

    bool initialized_;
    bool steady_state_;

    // The orientation as a Hamilton quaternion (q0 is the scalar). Represents
    // the orientation of the fixed frame wrt the body frame.
    double q0_, q1_, q2_, q3_;

    // Bias in angular velocities;
    double wx_prev_, wy_prev_, wz_prev_;

    // Bias in angular velocities;
    double wx_bias_, wy_bias_, wz_bias_;

    void updateBiases(double ax, double ay, double az, double wx, double wy,
                      double wz);

    bool checkState(double ax, double ay, double az, double wx, double wy,
                    double wz) const;

    void getPrediction(double wx, double wy, double wz, double dt,
                       double& q0_pred, double& q1_pred, double& q2_pred,
                       double& q3_pred) const;

    void getMeasurement(double ax, double ay, double az, double& q0_meas,
                        double& q1_meas, double& q2_meas, double& q3_meas);

    void getMeasurement(double ax, double ay, double az, double mx, double my,
                        double mz, double& q0_meas, double& q1_meas,
                        double& q2_meas, double& q3_meas);

    void getAccCorrection(double ax, double ay, double az, double p0, double p1,
                          double p2, double p3, double& dq0, double& dq1,
                          double& dq2, double& dq3);

    void getMagCorrection(double mx, double my, double mz, double p0, double p1,
                          double p2, double p3, double& dq0, double& dq1,
                          double& dq2, double& dq3);

    double getAdaptiveGain(double alpha, double ax, double ay, double az);
};

// Utility math functions:

void normalizeVector(double& x, double& y, double& z);

void normalizeQuaternion(double& q0, double& q1, double& q2, double& q3);

void scaleQuaternion(double gain, double& dq0, double& dq1, double& dq2,
                     double& dq3);

void invertQuaternion(double q0, double q1, double q2, double q3,
                      double& q0_inv, double& q1_inv, double& q2_inv,
                      double& q3_inv);

void quaternionMultiplication(double p0, double p1, double p2, double p3,
                              double q0, double q1, double q2, double q3,
                              double& r0, double& r1, double& r2, double& r3);

void rotateVectorByQuaternion(double x, double y, double z, double q0,
                              double q1, double q2, double q3, double& vx,
                              double& vy, double& vz);

}  // namespace imu_tools

#endif  // IMU_TOOLS_COMPLEMENTARY_FILTER_H
