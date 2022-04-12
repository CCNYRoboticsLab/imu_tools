
#ifndef TEST_TEST_HELPERS_H_
#define TEST_TEST_HELPERS_H_

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#define MAX_DIFF 0.05

template <typename T>
static inline void normalize_quaternion(T& q0, T& q1, T& q2, T& q3)
{
    T invNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    T max = q0;
    if (fabs(max) < fabs(q1)) max = q1;
    if (fabs(max) < fabs(q2)) max = q2;
    if (fabs(max) < fabs(q3)) max = q3;
    if (max < 0) invNorm *= -1.0;

    q0 *= invNorm;
    q1 *= invNorm;
    q2 *= invNorm;
    q3 *= invNorm;
}

// Tests for normalization in the same way as tf2:
// https://github.com/ros/geometry2/blob/bd490515b1434caeff521ea14901dfe04063ca27/tf2/src/buffer_core.cpp#L244-L247
template <typename T>
static inline bool is_normalized(T q0, T q1, T q2, T q3)
{
    return std::abs((q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3) - 1.0f) < 10e-6;
}

template <typename T>
static inline bool quat_equal(T q0, T q1, T q2, T q3, T qr0, T qr1, T qr2,
                              T qr3)
{
    normalize_quaternion(q0, q1, q2, q3);
    normalize_quaternion(qr0, qr1, qr2, qr3);

    return (fabs(q0 - qr0) < MAX_DIFF) && (fabs(q1 - qr1) < MAX_DIFF) &&
           (fabs(q2 - qr2) < MAX_DIFF) && (fabs(q3 - qr3) < MAX_DIFF);
}

template <typename T>
static inline bool quat_eq_ex_z(T q0, T q1, T q2, T q3, T qr0, T qr1, T qr2,
                                T qr3)
{
    // assert q == qr * qz
    tf2::Quaternion q(q1, q2, q3, q0);
    tf2::Quaternion qr(qr1, qr2, qr3, qr0);
    tf2::Quaternion qz = q * qr.inverse();

    // remove x and y components.
    qz.setX(0.0);
    qz.setY(0.0);

    tf2::Quaternion qr_ = qz * qr;

    return quat_equal(q0, q1, q2, q3, qr_.getW(), qr_.getX(), qr_.getY(),
                      qr_.getZ());
}

#define ASSERT_IS_NORMALIZED_(q0, q1, q2, q3)  \
    ASSERT_TRUE(is_normalized(q0, q1, q2, q3)) \
        << "q0: " << q0 << ", q1: " << q1 << ", q2: " << q2 << ", q3: " << q3;
#define ASSERT_IS_NORMALIZED(...) ASSERT_IS_NORMALIZED_(__VA_ARGS__)

#define ASSERT_QUAT_EQUAL_(q0, q1, q2, q3, qr0, qr1, qr2, qr3)  \
    ASSERT_TRUE(quat_equal(q0, q1, q2, q3, qr0, qr1, qr2, qr3)) \
        << "q0: " << q0 << ", q1: " << q1 << ", q2: " << q2 << ", q3: " << q3;
#define ASSERT_QUAT_EQUAL(...) ASSERT_QUAT_EQUAL_(__VA_ARGS__)

#define ASSERT_QUAT_EQUAL_EX_Z_(q0, q1, q2, q3, qr0, qr1, qr2, qr3) \
    ASSERT_TRUE(quat_eq_ex_z(q0, q1, q2, q3, qr0, qr1, qr2, qr3))   \
        << "q0: " << q0 << ", q1: " << q1 << ", q2: " << q2 << ", q3: " << q3;
#define ASSERT_QUAT_EQUAL_EX_Z(...) ASSERT_QUAT_EQUAL_EX_Z_(__VA_ARGS__)

// Well known states
// scheme: AM_x_y_z
#define AM_EAST_NORTH_UP /* Acceleration */ \
    0.0, 0.0, 9.81, /* Magnetic */ 0.0, 0.0005, -0.0005
#define AM_NORTH_EAST_DOWN /* Acceleration */ \
    0.0, 0.0, -9.81, /* Magnetic */ 0.0005, 0.0, 0.0005
#define AM_NORTH_WEST_UP /* Acceleration */ \
    0.0, 0.0, 9.81, /* Magnetic */ 0.0005, 0.0, -0.0005
#define AM_SOUTH_UP_WEST /* Acceleration */ \
    0.0, 9.81, 0.0, /* Magnetic */ -0.0005, -0.0005, 0.0
#define AM_SOUTH_EAST_UP /* Acceleration */ \
    0.0, 0.0, 9.81, /* Magnetic */ -0.0005, 0.0, -0.0005
#define AM_WEST_NORTH_DOWN /* Acceleration */ \
    0.0, 0.0, -9.81, /* Magnetic */ 0.0, 0.0005, 0.0005

// Real sensor data
#define AM_WEST_NORTH_DOWN_RSD /* Acceleration */ \
    0.12, 0.29, -9.83, /* Magnetic */ 6.363e-06, 1.0908e-05, 4.2723e-05
#define AM_NE_NW_UP_RSD /* Acceleration */ \
    0.20, 0.55, 9.779, /* Magnetic */ 8.484e-06, 8.181e-06, -4.3329e-05

// Strip accelration from am
#define ACCEL_ONLY(ax, ay, az, mx, my, mz) ax, ay, az

// Well known quaternion
// scheme: QUAT_axis_angle
#define QUAT_IDENTITY 1.0, 0.0, 0.0, 0.0
#define QUAT_MZ_90 0.707107, 0.0, 0.0, -0.707107
#define QUAT_X_180 0.0, 1.0, 0.0, 0.0
#define QUAT_XMYMZ_120 0.5, 0.5, -0.5, -0.5

#define QUAT_WEST_NORTH_DOWN_RSD_NWU 0.01, 0.86, 0.50, 0.012
/* axis: (0.864401, 0.502559, 0.0120614) | angle: 178.848deg */

#define QUAT_WEST_NORTH_DOWN_RSD_ENU 0.0, -0.25, -0.97, -0.02
/* Axis: (-0.2, -0.96, 0.02), Angle: 180deg */

#define QUAT_WEST_NORTH_DOWN_RSD_NED 0.86, -0.01, 0.01, -0.50
/* Axis: -Z, Angle: 60deg */

#define QUAT_NE_NW_UP_RSD_NWU 0.91, 0.03, -0.02, -0.41
/* axis: (0.0300376, -0.020025, -0.410513) | angle: 48.6734deg */

#define QUAT_NE_NW_UP_RSD_ENU 0.93, 0.03, 0.0, 0.35
/* Axis: Z,  Angle: 41deg */

#define QUAT_NE_NW_UP_RSD_NED 0.021, -0.91, -0.41, 0.02
/* Axis: (0.9, 0.4, 0.0), Angle: 180deg */

#endif /* TEST_TEST_HELPERS_H_ */
