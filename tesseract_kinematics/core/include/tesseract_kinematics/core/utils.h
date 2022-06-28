/**
 * @file utils.h
 * @brief Kinematics utility functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_KINEMATICS_UTILS_H
#define TESSERACT_KINEMATICS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <console_bridge/console.h>
#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/kinematic_group.h>

namespace tesseract_kinematics
{
template <typename FloatType>
using VectorX = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

/**
 * @brief Numerically calculate a jacobian. This is mainly used for testing
 * @param jacobian (Return) The jacobian which gets filled out.
 * @param kin          The kinematics object
 * @param joint_values The joint values for which to calculate the jacobian
 * @param link_name    The link_name for which the jacobian should be calculated
 * @param link_point   The point on the link for which to calculate the jacobian
 */
inline static void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                     const Eigen::Isometry3d& change_base,
                                     const tesseract_kinematics::ForwardKinematics& kin,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const std::string& link_name,
                                     const Eigen::Ref<const Eigen::Vector3d>& link_point)
{
  Eigen::VectorXd njvals;
  double delta = 1e-8;
  tesseract_common::TransformMap poses = kin.calcFwdKin(joint_values);
  Eigen::Isometry3d pose = poses[link_name];
  pose = change_base * pose;

  for (int i = 0; i < static_cast<int>(joint_values.size()); ++i)
  {
    njvals = joint_values;
    njvals[i] += delta;
    Eigen::Isometry3d updated_pose = kin.calcFwdKin(njvals)[link_name];
    updated_pose = change_base * updated_pose;

    Eigen::Vector3d temp = pose * link_point;
    Eigen::Vector3d temp2 = updated_pose * link_point;
    jacobian(0, i) = (temp2.x() - temp.x()) / delta;
    jacobian(1, i) = (temp2.y() - temp.y()) / delta;
    jacobian(2, i) = (temp2.z() - temp.z()) / delta;

    Eigen::VectorXd omega = (pose.rotation() * tesseract_common::calcRotationalError(pose.rotation().transpose() *
                                                                                     updated_pose.rotation())) /
                            delta;
    jacobian(3, i) = omega(0);
    jacobian(4, i) = omega(1);
    jacobian(5, i) = omega(2);
  }
}

/**
 * @brief Numerically calculate a jacobian. This is mainly used for testing
 * @param jacobian (Return) The jacobian which gets filled out.
 * @param joint_group          The joint group object
 * @param joint_values The joint values for which to calculate the jacobian
 * @param link_name    The link_name for which the jacobian should be calculated
 * @param link_point   The point on the link for which to calculate the jacobian
 */
inline static void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                     const JointGroup& joint_group,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const std::string& link_name,
                                     const Eigen::Ref<const Eigen::Vector3d>& link_point)
{
  Eigen::VectorXd njvals;
  double delta = 1e-8;
  tesseract_common::TransformMap poses = joint_group.calcFwdKin(joint_values);
  Eigen::Isometry3d pose = poses[link_name];

  for (int i = 0; i < static_cast<int>(joint_values.size()); ++i)
  {
    njvals = joint_values;
    njvals[i] += delta;
    tesseract_common::TransformMap updated_poses = joint_group.calcFwdKin(njvals);
    Eigen::Isometry3d updated_pose = updated_poses[link_name];

    Eigen::Vector3d temp = pose * link_point;
    Eigen::Vector3d temp2 = updated_pose * link_point;
    jacobian(0, i) = (temp2.x() - temp.x()) / delta;
    jacobian(1, i) = (temp2.y() - temp.y()) / delta;
    jacobian(2, i) = (temp2.z() - temp.z()) / delta;

    Eigen::VectorXd omega = (pose.rotation() * tesseract_common::calcRotationalError(pose.rotation().transpose() *
                                                                                     updated_pose.rotation())) /
                            delta;
    jacobian(3, i) = omega(0);
    jacobian(4, i) = omega(1);
    jacobian(5, i) = omega(2);
  }
}

/**
 * @brief Solve equation Ax=b for x
 * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
 * @param A Input matrix (represents Jacobian)
 * @param b Input vector (represents desired pose)
 * @param x Output vector (represents joint values)
 * @return True if solver completes properly
 */
inline static bool solvePInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
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
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
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

/**
 * @brief Calculate Damped Pseudoinverse
 * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
 * @param A Input matrix (represents Jacobian)
 * @param P Output matrix (represents pseudoinverse of A)
 * @param eps Singular value threshold
 * @param lambda Damping factor
 * @return True if Pseudoinverse completes properly
 */
inline static bool dampedPInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                              Eigen::Ref<Eigen::MatrixXd> P,
                              const double eps = 0.011,
                              const double lambda = 0.01)
{
  if ((A.rows() == 0) || (A.cols() == 0))
  {
    CONSOLE_BRIDGE_logError("Empty matrices not supported in dampedPInv()");
    return false;
  }

  // Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are
  // real)
  // in order to solve Ax=b -> x*=A+ b
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
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

/**
 * @brief Check if the provided jacobian is near a singularity
 * @details This is keep separated from the forward kinematics because special consideration may need to be made
 * based on the kinematics arrangement.
 * @param jacobian The jacobian to check if near a singularity
 * @param threshold The threshold that all singular values must be greater than or equal to not be considered near a
 * singularity
 */
inline bool isNearSingularity(const Eigen::Ref<const Eigen::MatrixXd>& jacobian, double threshold = 0.01)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd& sv = svd.singularValues();
  return (sv.tail(1).value() < threshold);
}

/** @brief Used to store Manipulability and Force Ellipsoid data */
struct ManipulabilityEllipsoid
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  /** @brief The manipulability ellipsoid eigen values */
  Eigen::VectorXd eigen_values;

  /**
   * @brief The ratio of longest and shortes axes of the manipulability ellipsoid
   * @details As this grows large it is approaching a singularity
   *   - measure = sqrt(max eigen value) / sqrt(min eigen value)
   *   - This should be greater than or equal to 1
   *   - If equal to 1 it is isotropic
   */
  double measure{ 0 };

  /**
   * @brief The condition number of A
   * @details
   *   - condition = (max eigen value) / (min eigen value)
   */
  double condition{ 0 };

  /** @brief This is propotial to the volume */
  double volume{ 0 };
};

/** @brief Contains both manipulability ellipsoid and force ellipsoid data */
struct Manipulability
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  /** @brief Full Manipulability Ellipsoid */
  ManipulabilityEllipsoid m;

  /** @brief Linear velocity manipulability ellipsoid */
  ManipulabilityEllipsoid m_linear;

  /** @brief Angular velocity manipulability ellipsoid */
  ManipulabilityEllipsoid m_angular;

  /** @brief Full Force Ellipsoid */
  ManipulabilityEllipsoid f;

  /** @brief Linear force manipulability ellipsoid */
  ManipulabilityEllipsoid f_linear;

  /** @brief Angular momentum manipulability ellipsoid */
  ManipulabilityEllipsoid f_angular;
};

/**
 * @brief Calculate manipulability data about the provided jacobian
 * @param jacobian The jacobian used to calculate manipulability
 * @return The manipulability data
 */
inline Manipulability calcManipulability(const Eigen::Ref<const Eigen::MatrixXd>& jacobian)
{
  Manipulability manip;
  Eigen::MatrixXd jacob_linear = jacobian.topRows(3);
  Eigen::MatrixXd jacob_angular = jacobian.bottomRows(3);

  auto fn = [](const Eigen::MatrixXd& m) {
    ManipulabilityEllipsoid data;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> sm(m, Eigen::DecompositionOptions::EigenvaluesOnly);
    data.eigen_values = sm.eigenvalues().real();

    // Set eigenvalues near zero to zero. This also implies zero volume
    for (Eigen::Index i = 0; i < data.eigen_values.size(); ++i)
    {
      if (tesseract_common::almostEqualRelativeAndAbs(data.eigen_values[i], 0))
        data.eigen_values[i] = +0;
    }

    // If the minimum eigen value is approximately zero set measure and condition to max double
    if (tesseract_common::almostEqualRelativeAndAbs(data.eigen_values.minCoeff(), 0))
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

  Eigen::MatrixXd a_inv = a.inverse();
  Eigen::MatrixXd a_linear_inv = a_linear.inverse();
  Eigen::MatrixXd a_angular_inv = a_angular.inverse();
  manip.f = fn(a_inv);
  manip.f_linear = fn(a_linear_inv);
  manip.f_angular = fn(a_angular_inv);

  return manip;
}

/**
 * @brief This a recursive function for calculating all permutations of the redundant solutions.
 * @details This should not be used directly, use getRedundantSolutions function.
 */
template <typename FloatType>
inline void getRedundantSolutionsHelper(std::vector<VectorX<FloatType>>& redundant_sols,
                                        const Eigen::Ref<const Eigen::VectorXd>& sol,
                                        const Eigen::MatrixX2d& limits,
                                        std::vector<Eigen::Index>::const_iterator current_index,
                                        std::vector<Eigen::Index>::const_iterator end_index)
{
  double val{ 0 };
  for (; current_index != end_index; ++current_index)
  {
    if (std::isinf(limits(*current_index, 0)))
    {
      std::stringstream ss;
      ss << "Lower limit of joint " << *current_index << " is infinite; no redundant solutions will be generated"
         << std::endl;
      CONSOLE_BRIDGE_logWarn(ss.str().c_str());
    }
    else
    {
      val = sol[*current_index];
      while ((val -= (2.0 * M_PI)) > limits(*current_index, 0) ||
             tesseract_common::almostEqualRelativeAndAbs(val, limits(*current_index, 0)))
      {
        // It not guaranteed that the provided solution is within limits so this check is needed
        if (val < limits(*current_index, 1) ||
            tesseract_common::almostEqualRelativeAndAbs(val, limits(*current_index, 1)))
        {
          Eigen::VectorXd new_sol = sol;
          new_sol[*current_index] = val;

          if (tesseract_common::satisfiesPositionLimits(new_sol, limits))
          {
            tesseract_common::enforcePositionLimits(new_sol, limits);
            redundant_sols.push_back(new_sol.template cast<FloatType>());
          }

          getRedundantSolutionsHelper<FloatType>(redundant_sols, new_sol, limits, current_index + 1, end_index);
        }
      }
    }

    if (std::isinf(limits(*current_index, 1)))
    {
      std::stringstream ss;
      ss << "Upper limit of joint " << *current_index << " is infinite; no redundant solutions will be generated"
         << std::endl;
      CONSOLE_BRIDGE_logWarn(ss.str().c_str());
    }
    else
    {
      val = sol[*current_index];
      while ((val += (2.0 * M_PI)) < limits(*current_index, 1) ||
             tesseract_common::almostEqualRelativeAndAbs(val, limits(*current_index, 1)))
      {
        // It not guaranteed that the provided solution is within limits so this check is needed
        if (val > limits(*current_index, 0) ||
            tesseract_common::almostEqualRelativeAndAbs(val, limits(*current_index, 0)))
        {
          Eigen::VectorXd new_sol = sol;
          new_sol[*current_index] = val;

          if (tesseract_common::satisfiesPositionLimits(new_sol, limits))
          {
            tesseract_common::enforcePositionLimits(new_sol, limits);
            redundant_sols.push_back(new_sol.template cast<FloatType>());
          }

          getRedundantSolutionsHelper<FloatType>(redundant_sols, new_sol, limits, current_index + 1, end_index);
        }
      }
    }
  }
}

/**
 * @brief Kinematics only return solution between PI and -PI. Provided the limits it will append redundant solutions.
 * @details The list of redundant solutions does not include the provided solutions.
 * @param sol The solution to calculate redundant solutions about
 * @param limits The joint limits of the robot
 * @param redundancy_capable_joints The indices of the redundancy capable joints
 */
template <typename FloatType>
inline std::vector<VectorX<FloatType>> getRedundantSolutions(const Eigen::Ref<const VectorX<FloatType>>& sol,
                                                             const Eigen::MatrixX2d& limits,
                                                             const std::vector<Eigen::Index>& redundancy_capable_joints)
{
  if (redundancy_capable_joints.empty())
    return {};

  for (const Eigen::Index& idx : redundancy_capable_joints)
  {
    if (idx >= sol.size())
    {
      std::stringstream ss;
      ss << "Redundant joint index " << idx << " is greater than or equal to the joint state size (" << sol.size()
         << ")";
      throw std::runtime_error(ss.str());
    }
  }

  std::vector<VectorX<FloatType>> redundant_sols;
  getRedundantSolutionsHelper<FloatType>(redundant_sols,
                                         sol.template cast<double>(),
                                         limits,
                                         redundancy_capable_joints.begin(),
                                         redundancy_capable_joints.end());
  return redundant_sols;
}

/**
 * @brief Given a vector of floats, this check if they are finite
 *
 *  In the case of OPW and IKFast they can return NANS so this is used to check that solutions are valid.
 *
 * @param qs A pointer to a floats array
 * @param dof The length of the float array
 * @return True if the array is valid, otherwise false
 */
template <typename FloatType>
inline bool isValid(const std::array<FloatType, 6>& qs)
{
  for (const auto& q : qs)
    if (!std::isfinite(q))
      return false;

  return true;
}

/**
 * @brief This take an array of floats and modifies them in place to be between [-PI, PI]
 * @param qs A pointer to a float array
 * @param dof The length of the float array
 */
template <typename FloatType>
DEPRECATED("Use redundancy version")
inline void harmonizeTowardZero(Eigen::Ref<VectorX<FloatType>> qs)
{
  const static auto pi = FloatType(M_PI);
  const static auto two_pi = FloatType(2.0 * M_PI);

  for (Eigen::Index i = 0; i < qs.rows(); ++i)
  {
    FloatType diff = std::fmod(qs[i] + pi, two_pi);
    qs[i] = (diff < 0) ? (diff + pi) : (diff - pi);
  }
}

/**
 * @brief This take an array of floats and modifies them in place to be between [-PI, PI]
 * @param qs A pointer to a float array
 * @param dof The length of the float array
 */
template <typename FloatType>
inline void harmonizeTowardZero(Eigen::Ref<VectorX<FloatType>> qs,
                                const std::vector<Eigen::Index>& redundancy_capable_joints)
{
  const static auto pi = FloatType(M_PI);
  const static auto two_pi = FloatType(2.0 * M_PI);

  for (const auto& i : redundancy_capable_joints)
  {
    FloatType diff = std::fmod(qs[i] + pi, two_pi);
    qs[i] = (diff < 0) ? (diff + pi) : (diff - pi);
  }
}

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_UTILS_H
