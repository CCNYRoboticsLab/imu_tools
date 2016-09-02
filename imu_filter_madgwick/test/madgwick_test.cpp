#include <imu_filter_madgwick/imu_filter.h>
#include <math.h>
#include "test_helpers.h"

#define FILTER_ITERATIONS 10000

void filterStationary(
    float Ax, float Ay, float Az,
    float Mx, float My, float Mz,
    double& q0, double& q1, double& q2, double& q3) {
  float dt = 0.1;
  float Gx = 0.0, Gy = 0.0, Gz = 0.0; // Stationary state => Gyro = (0,0,0)

  ImuFilter filter;
  filter.setDriftBiasGain(0.0);
  filter.setAlgorithmGain(0.1);

  // initialize with some orientation
  filter.setOrientation(q0,q1,q2,q3);

  for (int i = 0; i < FILTER_ITERATIONS; i++) {
      filter.madgwickAHRSupdate(Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz, dt);
  }

  filter.getOrientation(q0,q1,q2,q3);
}

#define TEST_STATIONARY_NWU(in_am, exp_result)       \
  TEST(MadgwickTest, Stationary_NED_ ## in_am){      \
    double q0 = .5, q1 = .5, q2 = .5, q3 = .5;       \
    filterStationary(in_am, q0, q1, q2, q3);  \
    ASSERT_QUAT_EQAL(q0, q1, q2, q3, exp_result); }

TEST_STATIONARY_NWU(AM_NORTH_EAST_DOWN, QUAT_X_180)
TEST_STATIONARY_NWU(AM_NORTH_WEST_UP, QUAT_IDENTITY)

// Real sensor data tests
TEST_STATIONARY_NWU(AM_WEST_NORTH_DOWN_RSD, QUAT_WEST_NORTH_DOWN_RSD_NWU)
TEST_STATIONARY_NWU(AM_NE_NW_UP_RSD, QUAT_NE_NW_UP_RSD_NWU)




int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
