/**
 * @file utils.cpp
 * @brief Kinematics utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/utils.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/forward_kinematics.h>

namespace tesseract::kinematics
{
void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                       const Eigen::Isometry3d& change_base,
                       const ForwardKinematics& kin,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                       const std::string& link_name,
                       const Eigen::Ref<const Eigen::Vector3d>& link_point)
{
  Eigen::VectorXd njvals;
  double delta = 1e-8;
  TESSERACT_THREAD_LOCAL tesseract::common::TransformMap poses;
  poses.clear();
  kin.calcFwdKin(poses, joint_values);
  Eigen::Isometry3d pose{ change_base * poses[link_name] };

  for (int i = 0; i < static_cast<int>(joint_values.size()); ++i)
  {
    njvals = joint_values;
    njvals[i] += delta;
    kin.calcFwdKin(poses, njvals);
    Eigen::Isometry3d updated_pose = change_base * poses[link_name];

    Eigen::Vector3d temp{ pose * link_point };
    Eigen::Vector3d temp2{ updated_pose * link_point };
    jacobian(0, i) = (temp2.x() - temp.x()) / delta;
    jacobian(1, i) = (temp2.y() - temp.y()) / delta;
    jacobian(2, i) = (temp2.z() - temp.z()) / delta;

    Eigen::Vector3d omega = (pose.rotation() * tesseract::common::calcRotationalError(pose.rotation().transpose() *
                                                                                      updated_pose.rotation())) /
                            delta;
    jacobian(3, i) = omega(0);
    jacobian(4, i) = omega(1);
    jacobian(5, i) = omega(2);
  }
}

void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                       const Eigen::Isometry3d& change_base,
                       const JointGroup& joint_group,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                       const std::string& link_name,
                       const Eigen::Ref<const Eigen::Vector3d>& link_point)
{
  Eigen::VectorXd njvals;
  constexpr double delta = 1e-8;
  tesseract::common::TransformMap poses = joint_group.calcFwdKin(joint_values);
  Eigen::Isometry3d pose = change_base * poses[link_name];

  for (int i = 0; i < static_cast<int>(joint_values.size()); ++i)
  {
    njvals = joint_values;
    njvals(i) += delta;  // NOLINT
    tesseract::common::TransformMap updated_poses = joint_group.calcFwdKin(njvals);
    Eigen::Isometry3d updated_pose = change_base * updated_poses[link_name];

    Eigen::Vector3d temp = pose * link_point;
    Eigen::Vector3d temp2 = updated_pose * link_point;
    jacobian(0, i) = (temp2.x() - temp.x()) / delta;
    jacobian(1, i) = (temp2.y() - temp.y()) / delta;
    jacobian(2, i) = (temp2.z() - temp.z()) / delta;

    Eigen::VectorXd omega = (pose.rotation() * tesseract::common::calcRotationalError(pose.rotation().transpose() *
                                                                                      updated_pose.rotation())) /
                            delta;
    jacobian(3, i) = omega(0);
    jacobian(4, i) = omega(1);
    jacobian(5, i) = omega(2);
  }
}

void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                       const JointGroup& joint_group,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                       const std::string& base_link_name,
                       const Eigen::Isometry3d& base_link_offset,
                       const std::string& link_name,
                       const Eigen::Isometry3d& link_offset)
{
  tesseract::common::TransformMap poses;

  joint_group.calcFwdKin(poses, joint_values);
  const Eigen::Isometry3d change_base = (poses[base_link_name] * base_link_offset).inverse();
  Eigen::MatrixXd base_jacobian(6, joint_group.numJoints());
  numericalJacobian(base_jacobian,
                    Eigen::Isometry3d::Identity(),
                    joint_group,
                    joint_values,
                    base_link_name,
                    base_link_offset.translation());
  tesseract::common::jacobianChangeBase(base_jacobian, change_base);

  Eigen::MatrixXd link_jacobian(6, joint_group.numJoints());
  numericalJacobian(
      link_jacobian, Eigen::Isometry3d::Identity(), joint_group, joint_values, link_name, link_offset.translation());
  tesseract::common::jacobianChangeBase(link_jacobian, change_base);

  jacobian.noalias() = link_jacobian - base_jacobian;
}

bool solvePInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::VectorXd>& b,
               Eigen::Ref<Eigen::VectorXd> x)
{
  const double eps = 0.00001;  // TODO: Turn into class member var
  const double lambda = 0.01;  // TODO: Turn into class member var

  if ((A.rows() == 0) || (A.cols() == 0))
  {
    CONSOLE_BRIDGE_logError("Empty matrices not supported in solvePinv()");
    return false;
  }

  if (A.rows() != b.size())
  {
    CONSOLE_BRIDGE_logError("Matrix size mismatch: A(%d, %d), b(%d)", A.rows(), A.cols(), b.size());
    return false;
  }

  // Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are
  // real)
  // in order to solve Ax=b -> x*=A+ b
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const Eigen::MatrixXd& U = svd.matrixU();
  const Eigen::VectorXd& Sv = svd.singularValues();
  const Eigen::MatrixXd& V = svd.matrixV();

  // calculate the reciprocal of Singular-Values
  // damp inverse with lambda so that inverse doesn't oscillate near solution
  long int nSv = Sv.size();
  Eigen::VectorXd inv_Sv(nSv);
  for (long int i = 0; i < nSv; ++i)
  {
    if (fabs(Sv(i)) > eps)
      inv_Sv(i) = 1 / Sv(i);
    else
      inv_Sv(i) = Sv(i) / (Sv(i) * Sv(i) + lambda * lambda);
  }
  x = V * inv_Sv.asDiagonal() * U.transpose() * b;
  return true;
}

bool dampedPInv(const Eigen::Ref<const Eigen::MatrixXd>& A, Eigen::Ref<Eigen::MatrixXd> P, double eps, double lambda)
{
  if ((A.rows() == 0) || (A.cols() == 0))
  {
    CONSOLE_BRIDGE_logError("Empty matrices not supported in dampedPInv()");
    return false;
  }

  // Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are
  // real)
  // in order to solve Ax=b -> x*=A+ b
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::MatrixXd& U = svd.matrixU();
  const Eigen::VectorXd& Sv = svd.singularValues();
  const Eigen::MatrixXd& V = svd.matrixV();

  // calculate the reciprocal of Singular-Values
  // damp inverse with lambda so that inverse doesn't oscillate near solution
  long int nSv = Sv.size();
  Eigen::VectorXd inv_Sv(nSv);
  for (long int i = 0; i < nSv; ++i)
  {
    if (fabs(Sv(i)) > eps)
      inv_Sv(i) = 1 / Sv(i);
    else
    {
      inv_Sv(i) = Sv(i) / (Sv(i) * Sv(i) + lambda * lambda);
    }
  }
  P = V * inv_Sv.asDiagonal() * U.transpose();
  return true;
}

bool isNearSingularity(const Eigen::Ref<const Eigen::MatrixXd>& jacobian, double threshold)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  svd.compute(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd& sv = svd.singularValues();
  return (sv.tail(1).value() < threshold);
}

Manipulability calcManipulability(const Eigen::Ref<const Eigen::MatrixXd>& jacobian)
{
  Manipulability manip;
  Eigen::MatrixXd jacob_linear = jacobian.topRows(3);
  Eigen::MatrixXd jacob_angular = jacobian.bottomRows(3);

  auto fn = [](const Eigen::MatrixXd& m) {
    ManipulabilityEllipsoid data;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> sm(m, Eigen::DecompositionOptions::EigenvaluesOnly);
    data.eigen_values = sm.eigenvalues().real();

    // Set eigenvalues near zero to zero. This also implies zero volume
    for (Eigen::Index i = 0; i < data.eigen_values.size(); ++i)  // NOLINT(modernize-loop-convert)
    {
      if (tesseract::common::almostEqualRelativeAndAbs(data.eigen_values[i], 0))
        data.eigen_values[i] = +0;
    }

    // If the minimum eigen value is approximately zero set measure and condition to max double
    if (tesseract::common::almostEqualRelativeAndAbs(data.eigen_values.minCoeff(), 0))
    {
      data.measure = std::numeric_limits<double>::max();
      data.condition = std::numeric_limits<double>::max();
    }
    else
    {
      data.condition = data.eigen_values.maxCoeff() / data.eigen_values.minCoeff();
      data.measure = std::sqrt(data.condition);
    }

    data.volume = std::sqrt(data.eigen_values.prod());

    return data;
  };

  Eigen::MatrixXd a = jacobian * jacobian.transpose();
  Eigen::MatrixXd a_linear = jacob_linear * jacob_linear.transpose();
  Eigen::MatrixXd a_angular = jacob_angular * jacob_angular.transpose();
  manip.m = fn(a);
  manip.m_linear = fn(a_linear);
  manip.m_angular = fn(a_angular);

  // NOLINTNEXTLINE
  Eigen::MatrixXd a_inv = a.inverse();
  Eigen::MatrixXd a_linear_inv = a_linear.inverse();
  Eigen::MatrixXd a_angular_inv = a_angular.inverse();
  manip.f = fn(a_inv);
  manip.f_linear = fn(a_linear_inv);
  manip.f_angular = fn(a_angular_inv);

  return manip;
}
}  // namespace tesseract::kinematics
