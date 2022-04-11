#include <imu_filter_madgwick/stateless_orientation.h>
#include "test_helpers.h"

template <WorldFrame::WorldFrame FRAME>
bool computeOrientation(float Ax, float Ay, float Az, float Mx, float My,
                        float Mz, double& q0, double& q1, double& q2,
                        double& q3)
{
    geometry_msgs::msg::Vector3 A, M;
    geometry_msgs::msg::Quaternion orientation;
    A.x = Ax;
    A.y = Ay;
    A.z = Az;
    M.x = Mx;
    M.y = My;
    M.z = Mz;

    bool res =
        StatelessOrientation::computeOrientation(FRAME, A, M, orientation);

    q0 = orientation.w;
    q1 = orientation.x;
    q2 = orientation.y;
    q3 = orientation.z;

    return res;
}

template <WorldFrame::WorldFrame FRAME>
bool computeOrientation(float Ax, float Ay, float Az, double& q0, double& q1,
                        double& q2, double& q3)
{
    geometry_msgs::msg::Vector3 A;
    geometry_msgs::msg::Quaternion orientation;
    A.x = Ax;
    A.y = Ay;
    A.z = Az;

    bool res = StatelessOrientation::computeOrientation(FRAME, A, orientation);

    q0 = orientation.w;
    q1 = orientation.x;
    q2 = orientation.y;
    q3 = orientation.z;

    return res;
}

#define TEST_STATELESS_ENU(in_am, exp_result)                                  \
    TEST(StatelessOrientationTest, Stationary_ENU_##in_am)                     \
    {                                                                          \
        double q0, q1, q2, q3;                                                 \
        ASSERT_TRUE(                                                           \
            computeOrientation<WorldFrame::ENU>(in_am, q0, q1, q2, q3));       \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                  \
        ASSERT_QUAT_EQUAL(q0, q1, q2, q3, exp_result);                         \
    }                                                                          \
    TEST(StatelessOrientationTest, Stationary_ENU_NM_##in_am)                  \
    {                                                                          \
        double q0, q1, q2, q3;                                                 \
        ASSERT_TRUE(computeOrientation<WorldFrame::ENU>(ACCEL_ONLY(in_am), q0, \
                                                        q1, q2, q3));          \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                  \
        ASSERT_QUAT_EQUAL_EX_Z(q0, q1, q2, q3, exp_result);                    \
    }

#define TEST_STATELESS_NED(in_am, exp_result)                                  \
    TEST(StatelessOrientationTest, Stationary_NED_##in_am)                     \
    {                                                                          \
        double q0, q1, q2, q3;                                                 \
        ASSERT_TRUE(                                                           \
            computeOrientation<WorldFrame::NED>(in_am, q0, q1, q2, q3));       \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                  \
        ASSERT_QUAT_EQUAL(q0, q1, q2, q3, exp_result);                         \
    }                                                                          \
    TEST(StatelessOrientationTest, Stationary_NED_NM_##in_am)                  \
    {                                                                          \
        double q0, q1, q2, q3;                                                 \
        ASSERT_TRUE(computeOrientation<WorldFrame::NED>(ACCEL_ONLY(in_am), q0, \
                                                        q1, q2, q3));          \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                  \
        ASSERT_QUAT_EQUAL_EX_Z(q0, q1, q2, q3, exp_result);                    \
    }

#define TEST_STATELESS_NWU(in_am, exp_result)                                  \
    TEST(StatelessOrientationTest, Stationary_NWU_##in_am)                     \
    {                                                                          \
        double q0, q1, q2, q3;                                                 \
        ASSERT_TRUE(                                                           \
            computeOrientation<WorldFrame::NWU>(in_am, q0, q1, q2, q3));       \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                  \
        ASSERT_QUAT_EQUAL(q0, q1, q2, q3, exp_result);                         \
    }                                                                          \
    TEST(StatelessOrientationTest, Stationary_NWU_NM_##in_am)                  \
    {                                                                          \
        double q0, q1, q2, q3;                                                 \
        ASSERT_TRUE(computeOrientation<WorldFrame::NWU>(ACCEL_ONLY(in_am), q0, \
                                                        q1, q2, q3));          \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                  \
        ASSERT_QUAT_EQUAL_EX_Z(q0, q1, q2, q3, exp_result);                    \
    }

TEST_STATELESS_ENU(AM_EAST_NORTH_UP, QUAT_IDENTITY)
TEST_STATELESS_ENU(AM_SOUTH_UP_WEST, QUAT_XMYMZ_120)
TEST_STATELESS_ENU(AM_SOUTH_EAST_UP, QUAT_MZ_90)
TEST_STATELESS_ENU(AM_WEST_NORTH_DOWN_RSD, QUAT_WEST_NORTH_DOWN_RSD_ENU)
TEST_STATELESS_ENU(AM_NE_NW_UP_RSD, QUAT_NE_NW_UP_RSD_ENU)

TEST_STATELESS_NED(AM_NORTH_EAST_DOWN, QUAT_IDENTITY)
TEST_STATELESS_NED(AM_WEST_NORTH_DOWN, QUAT_MZ_90)
TEST_STATELESS_NED(AM_WEST_NORTH_DOWN_RSD, QUAT_WEST_NORTH_DOWN_RSD_NED)
TEST_STATELESS_NED(AM_NE_NW_UP_RSD, QUAT_NE_NW_UP_RSD_NED)

TEST_STATELESS_NWU(AM_NORTH_EAST_DOWN, QUAT_X_180)
TEST_STATELESS_NWU(AM_NORTH_WEST_UP, QUAT_IDENTITY)
TEST_STATELESS_NWU(AM_WEST_NORTH_DOWN_RSD, QUAT_WEST_NORTH_DOWN_RSD_NWU)
TEST_STATELESS_NWU(AM_NE_NW_UP_RSD, QUAT_NE_NW_UP_RSD_NWU)

TEST(StatelessOrientationTest, Check_NoAccel)
{
    double q0, q1, q2, q3;
    ASSERT_FALSE(computeOrientation<WorldFrame::ENU>(0.0, 0.0, 0.0, 0.0, 0.0005,
                                                     -0.0005, q0, q1, q2, q3));
}

TEST(StatelessOrientationTest, Check_NoMag)
{
    double q0, q1, q2, q3;
    ASSERT_FALSE(computeOrientation<WorldFrame::ENU>(0.0, 0.0, 9.81, 0.0, 0.0,
                                                     0.0, q0, q1, q2, q3));
}
