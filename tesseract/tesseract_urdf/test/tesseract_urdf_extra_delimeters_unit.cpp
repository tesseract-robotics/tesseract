#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/origin.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_extra_delimeters)  // NOLINT
{
  {
    std::string str = R"(<origin xyz="0   2.5   0" rpy="3.14159265359   0  0.0"/>)";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.translation().isApprox(Eigen::Vector3d(0, 2.5, 0), 1e-8));
    EXPECT_TRUE(origin.matrix().col(0).head(3).isApprox(Eigen::Vector3d(1, 0, 0), 1e-8));
    EXPECT_TRUE(origin.matrix().col(1).head(3).isApprox(Eigen::Vector3d(0, -1, 0), 1e-8));
    EXPECT_TRUE(origin.matrix().col(2).head(3).isApprox(Eigen::Vector3d(0, 0, -1), 1e-8));
  }
}
