#include <gtest/gtest.h>
#include <tesseract/common/kinematic_limits.h>
TEST(TesseractCommonUnit, KinematicLimitsEquality)  // NOLINT
{
  using tesseract::common::KinematicLimits;
  KinematicLimits empty;
  EXPECT_EQ(empty, KinematicLimits());

  KinematicLimits limits;
  limits.resize(2);
  limits.joint_limits.setOnes();
  limits.velocity_limits.setConstant(2);
  limits.acceleration_limits.setConstant(3);
  limits.jerk_limits.setConstant(4);
  EXPECT_EQ(limits, limits);
  EXPECT_NE(limits, empty);
  EXPECT_NE(empty, limits);

  for (auto member : { &KinematicLimits::joint_limits, &KinematicLimits::velocity_limits,
                       &KinematicLimits::acceleration_limits, &KinematicLimits::jerk_limits })
  {
    for (Eigen::Index rows : { 0, 1, 3 })
    {
      KinematicLimits other = limits;
      (other.*member).resize(rows, 2);
      (other.*member).setOnes();
      EXPECT_NE(limits, other);
      EXPECT_NE(other, limits);
    }

    KinematicLimits other = limits;
    (other.*member)(0, 0) += 1e-7;
    EXPECT_EQ(limits, other);
    EXPECT_EQ(other, limits);
    (other.*member)(0, 0) += 1;
    EXPECT_NE(limits, other);
    EXPECT_NE(other, limits);
  }
}
