#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/origin.h>
#include "tesseract_urdf_common_unit.h"

/**
 * @brief This function was pulled from urdfdom library to verify that the method of using Eigen produces
 * the same result. This is only used for this unit test and Eigen is used within the origin class to
 * convert roll, pitch and yaw to a rotation matrix.
 */
Eigen::Quaterniond fromRPY(double roll, double pitch, double yaw)
{
  double phi, the, psi;

  phi = roll / 2.0;
  the = pitch / 2.0;
  psi = yaw / 2.0;

  double x, y, z, w;
  x = std::sin(phi) * std::cos(the) * std::cos(psi) - std::cos(phi) * std::sin(the) * std::sin(psi);
  y = std::cos(phi) * std::sin(the) * std::cos(psi) + std::sin(phi) * std::cos(the) * std::sin(psi);
  z = std::cos(phi) * std::cos(the) * std::sin(psi) - std::sin(phi) * std::sin(the) * std::cos(psi);
  w = std::cos(phi) * std::cos(the) * std::cos(psi) + std::sin(phi) * std::sin(the) * std::sin(psi);

  return Eigen::Quaterniond(w, x, y, z);
}

TEST(TesseractURDFUnit, parse_origin)  // NOLINT
{
  {
    std::string str = R"(<origin xyz="0 0 0" rpy="0 0 0"/>)";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<origin xyz="0 0 0" wxyz="1 0 0 0"/>)";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<origin xyz="0 0 0" wxyz="0.8719632 0.247934 0.177848 0.3828563"/>)";
    Eigen::Isometry3d origin;
    Eigen::Isometry3d check = Eigen::Isometry3d::Identity();
    check.linear() << 0.6435823, -0.5794841, 0.5000000, 0.7558624, 0.5838996, -0.2961981, -0.1203077, 0.5685591,
        0.8137977;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(check, 1e-6));
  }

  {
    std::string str = R"(<origin xyz="0 0 0" rpy="0.3490659 0.5235988 0.7330383"/>)";
    Eigen::Isometry3d origin;
    Eigen::Isometry3d check = Eigen::Isometry3d::Identity();
    check.linear() = fromRPY(0.3490659, 0.5235988, 0.7330383).toRotationMatrix();
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(check, 1e-6));
  }

  {
    std::string str = R"(<origin xyz="0 2.5 0" rpy="3.14159265359 0 0"/>)";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.translation().isApprox(Eigen::Vector3d(0, 2.5, 0), 1e-8));
    EXPECT_TRUE(origin.matrix().col(0).head(3).isApprox(Eigen::Vector3d(1, 0, 0), 1e-8));
    EXPECT_TRUE(origin.matrix().col(1).head(3).isApprox(Eigen::Vector3d(0, -1, 0), 1e-8));
    EXPECT_TRUE(origin.matrix().col(2).head(3).isApprox(Eigen::Vector3d(0, 0, -1), 1e-8));

    Eigen::Quaterniond check_q = fromRPY(3.14159265359, 0, 0);
    Eigen::Quaterniond orig_q(origin.rotation());

    EXPECT_TRUE(check_q.matrix().isApprox(orig_q.matrix(), 1e-8));
  }

  {
    std::string str = R"(<origin xyz="0 2.5 0" rpy="3.14 0 0" wxyz="1 0 0 0"/>)";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.translation().isApprox(Eigen::Vector3d(0, 2.5, 0), 1e-8));
    EXPECT_TRUE(origin.rotation().isApprox(Eigen::Matrix3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<origin xyz="0 0 0"/>)";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<origin rpy="0 0 0"/>)";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(origin.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    std::string str = R"(<origin />)";
    Eigen::Isometry3d origin;
    auto status = runTest<Eigen::Isometry3d>(origin, str, "origin", 2);
    EXPECT_FALSE(*status);
  }
}
