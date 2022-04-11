#include <imu_filter_madgwick/imu_filter.h>
#include <cmath>
#include "test_helpers.h"

#define FILTER_ITERATIONS 10000

template <WorldFrame::WorldFrame FRAME>
void filterStationary(float Ax, float Ay, float Az, float Mx, float My,
                      float Mz, double& q0, double& q1, double& q2, double& q3)
{
    float dt = 0.1;
    float Gx = 0.0, Gy = 0.0, Gz = 0.0;  // Stationary state => Gyro = (0,0,0)

    ImuFilter filter;
    filter.setDriftBiasGain(0.0);
    filter.setAlgorithmGain(0.1);

    // initialize with some orientation
    filter.setOrientation(q0, q1, q2, q3);
    filter.setWorldFrame(FRAME);

    for (int i = 0; i < FILTER_ITERATIONS; i++)
    {
        filter.madgwickAHRSupdate(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz, dt);
    }

    filter.getOrientation(q0, q1, q2, q3);
}

template <WorldFrame::WorldFrame FRAME>
void filterStationary(float Ax, float Ay, float Az, double& q0, double& q1,
                      double& q2, double& q3)
{
    float dt = 0.1;
    float Gx = 0.0, Gy = 0.0, Gz = 0.0;  // Stationary state => Gyro = (0,0,0)

    ImuFilter filter;
    filter.setDriftBiasGain(0.0);
    filter.setAlgorithmGain(0.1);

    // initialize with some orientation
    filter.setOrientation(q0, q1, q2, q3);
    filter.setWorldFrame(FRAME);

    for (int i = 0; i < FILTER_ITERATIONS; i++)
    {
        filter.madgwickAHRSupdateIMU(Gx, Gy, Gz, Ax, Ay, Az, dt);
    }

    filter.getOrientation(q0, q1, q2, q3);
}

#define TEST_STATIONARY_ENU(in_am, exp_result)                                \
    TEST(MadgwickTest, Stationary_ENU_##in_am)                                \
    {                                                                         \
        double q0 = .5, q1 = .5, q2 = .5, q3 = .5;                            \
        filterStationary<WorldFrame::ENU>(in_am, q0, q1, q2, q3);             \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                 \
        ASSERT_QUAT_EQUAL(q0, q1, q2, q3, exp_result);                        \
    }                                                                         \
    TEST(MadgwickTest, Stationary_ENU_NM_##in_am)                             \
    {                                                                         \
        double q0 = .5, q1 = .5, q2 = .5, q3 = .5;                            \
        filterStationary<WorldFrame::ENU>(ACCEL_ONLY(in_am), q0, q1, q2, q3); \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                 \
        ASSERT_QUAT_EQUAL_EX_Z(q0, q1, q2, q3, exp_result);                   \
    }

#define TEST_STATIONARY_NED(in_am, exp_result)                                \
    TEST(MadgwickTest, Stationary_NED_##in_am)                                \
    {                                                                         \
        double q0 = .5, q1 = .5, q2 = .5, q3 = .5;                            \
        filterStationary<WorldFrame::NED>(in_am, q0, q1, q2, q3);             \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                 \
        ASSERT_QUAT_EQUAL(q0, q1, q2, q3, exp_result);                        \
    }                                                                         \
    TEST(MadgwickTest, Stationary_NED_NM_##in_am)                             \
    {                                                                         \
        double q0 = .5, q1 = .5, q2 = .5, q3 = .5;                            \
        filterStationary<WorldFrame::NED>(ACCEL_ONLY(in_am), q0, q1, q2, q3); \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                 \
        ASSERT_QUAT_EQUAL_EX_Z(q0, q1, q2, q3, exp_result);                   \
    }

#define TEST_STATIONARY_NWU(in_am, exp_result)                                \
    TEST(MadgwickTest, Stationary_NWU_##in_am)                                \
    {                                                                         \
        double q0 = .5, q1 = .5, q2 = .5, q3 = .5;                            \
        filterStationary<WorldFrame::NWU>(in_am, q0, q1, q2, q3);             \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                 \
        ASSERT_QUAT_EQUAL(q0, q1, q2, q3, exp_result);                        \
    }                                                                         \
    TEST(MadgwickTest, Stationary_NWU_NM_##in_am)                             \
    {                                                                         \
        double q0 = .5, q1 = .5, q2 = .5, q3 = .5;                            \
        filterStationary<WorldFrame::NWU>(ACCEL_ONLY(in_am), q0, q1, q2, q3); \
        ASSERT_IS_NORMALIZED(q0, q1, q2, q3);                                 \
        ASSERT_QUAT_EQUAL_EX_Z(q0, q1, q2, q3, exp_result);                   \
    }

TEST_STATIONARY_NWU(AM_NORTH_EAST_DOWN, QUAT_X_180)
TEST_STATIONARY_NWU(AM_NORTH_WEST_UP, QUAT_IDENTITY)
TEST_STATIONARY_NWU(AM_WEST_NORTH_DOWN_RSD, QUAT_WEST_NORTH_DOWN_RSD_NWU)
TEST_STATIONARY_NWU(AM_NE_NW_UP_RSD, QUAT_NE_NW_UP_RSD_NWU)

TEST_STATIONARY_ENU(AM_EAST_NORTH_UP, QUAT_IDENTITY)
TEST_STATIONARY_ENU(AM_SOUTH_UP_WEST, QUAT_XMYMZ_120)
TEST_STATIONARY_ENU(AM_SOUTH_EAST_UP, QUAT_MZ_90)
TEST_STATIONARY_ENU(AM_WEST_NORTH_DOWN_RSD, QUAT_WEST_NORTH_DOWN_RSD_ENU)
TEST_STATIONARY_ENU(AM_NE_NW_UP_RSD, QUAT_NE_NW_UP_RSD_ENU)

TEST_STATIONARY_NED(AM_NORTH_EAST_DOWN, QUAT_IDENTITY)
TEST_STATIONARY_NED(AM_NORTH_WEST_UP, QUAT_X_180)
TEST_STATIONARY_NED(AM_WEST_NORTH_DOWN_RSD, QUAT_WEST_NORTH_DOWN_RSD_NED)
TEST_STATIONARY_NED(AM_NE_NW_UP_RSD, QUAT_NE_NW_UP_RSD_NED)

TEST(MadgwickTest, TestQuatEqNoZ)
{
    ASSERT_TRUE(quat_eq_ex_z(QUAT_IDENTITY, QUAT_MZ_90));
    ASSERT_FALSE(quat_eq_ex_z(QUAT_IDENTITY, QUAT_X_180));
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
