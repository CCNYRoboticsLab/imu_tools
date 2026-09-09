/*
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <gtest/gtest.h>

#include <cmath>

#include "imu_complementary_filter/complementary_filter.h"

namespace {

constexpr double kTolerance = 1e-12;
constexpr double kGain = 0.25;

TEST(ScaleQuaternion, UsesSlerpForLargeCorrections)
{
    double q0 = 0.8;
    double q1 = 0.6;
    double q2 = 0.0;
    double q3 = 0.0;

    const double angle = std::acos(q0);
    const double expected_q0 = std::cos(kGain * angle);
    const double expected_q1 = std::sin(kGain * angle);

    imu_tools::scaleQuaternion(kGain, q0, q1, q2, q3);

    EXPECT_NEAR(q0, expected_q0, kTolerance);
    EXPECT_NEAR(q1, expected_q1, kTolerance);
    EXPECT_DOUBLE_EQ(q2, 0.0);
    EXPECT_DOUBLE_EQ(q3, 0.0);
}

TEST(ScaleQuaternion, UsesNormalizedLerpForSmallCorrections)
{
    double q0 = 0.95;
    double q1 = std::sqrt(1.0 - q0 * q0);
    double q2 = 0.0;
    double q3 = 0.0;

    const double lerp_q0 = (1.0 - kGain) + kGain * q0;
    const double lerp_q1 = kGain * q1;
    const double norm = std::hypot(lerp_q0, lerp_q1);

    imu_tools::scaleQuaternion(kGain, q0, q1, q2, q3);

    EXPECT_NEAR(q0, lerp_q0 / norm, kTolerance);
    EXPECT_NEAR(q1, lerp_q1 / norm, kTolerance);
    EXPECT_DOUBLE_EQ(q2, 0.0);
    EXPECT_DOUBLE_EQ(q3, 0.0);
}

}  // namespace
