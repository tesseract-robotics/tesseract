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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <console_bridge/console.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_scene_graph/graph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_kinematics
{
/**
 * @brief Change the base coordinate system of the jacobian
 * @param jacobian The current Jacobian which gets modified in place
 * @param change_base The transform from the desired frame to the current base frame of the jacobian
 */
inline static void jacobianChangeBase(Eigen::Ref<Eigen::MatrixXd> jacobian, const Eigen::Isometry3d& change_base)
{
  assert(jacobian.rows() == 6);
  for (int i = 0; i < jacobian.cols(); i++)
  {
    jacobian.col(i).head(3) = change_base.linear() * jacobian.col(i).head(3);
    jacobian.col(i).tail(3) = change_base.linear() * jacobian.col(i).tail(3);
  }
}

/**
 * @brief Change the reference point of the jacobian
 * @param jacobian The current Jacobian which gets modified in place
 * @param ref_point Is expressed in the same base frame of the jacobian
 *                  and is a vector from the old point to the new point.
 */
inline static void jacobianChangeRefPoint(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                          const Eigen::Ref<const Eigen::Vector3d>& ref_point)
{
  assert(jacobian.rows() == 6);
  for (int i = 0; i < jacobian.cols(); i++)
  {
    jacobian(0, i) += jacobian(4, i) * ref_point(2) - jacobian(5, i) * ref_point(1);
    jacobian(1, i) += jacobian(5, i) * ref_point(0) - jacobian(3, i) * ref_point(2);
    jacobian(2, i) += jacobian(3, i) * ref_point(1) - jacobian(4, i) * ref_point(0);
  }
}

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
  double delta = 0.001;
  Eigen::Isometry3d pose;
  kin.calcFwdKin(pose, joint_values, link_name);
  pose = change_base * pose;

  for (int i = 0; i < static_cast<int>(joint_values.size()); ++i)
  {
    njvals = joint_values;
    njvals[i] += delta;
    Eigen::Isometry3d updated_pose;
    kin.calcFwdKin(updated_pose, njvals, link_name);
    updated_pose = change_base * updated_pose;

    Eigen::Vector3d temp = pose * link_point;
    Eigen::Vector3d temp2 = updated_pose * link_point;
    jacobian(0, i) = (temp2.x() - temp.x()) / delta;
    jacobian(1, i) = (temp2.y() - temp.y()) / delta;
    jacobian(2, i) = (temp2.z() - temp.z()) / delta;

    Eigen::AngleAxisd r12(pose.rotation().transpose() * updated_pose.rotation());  // rotation from p1 -> p2
    double theta = r12.angle();
    theta = copysign(fmod(fabs(theta), 2.0 * M_PI), theta);
    if (theta < -M_PI)
      theta = theta + 2. * M_PI;
    if (theta > M_PI)
      theta = theta - 2. * M_PI;
    Eigen::VectorXd omega = (pose.rotation() * r12.axis() * theta) / delta;
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
 * @brief Create a kinematics map from the srdf model
 * @param scene_graph Tesseract Scene Graph
 * @param srdf_model Tesseract SRDF Model
 * @return Kinematics map between group name and kinematics object
 */
template <class Chain_T, class Tree_T>
ForwardKinematicsConstPtrMap createKinematicsMap(const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
                                                 const tesseract_scene_graph::SRDFModel& srdf_model)
{
  ForwardKinematicsConstPtrMap manipulators;
  for (const auto& group : srdf_model.getGroups())
  {
    if (!group.chains_.empty())
    {
      assert(group.chains_.size() == 1);
      if (manipulators.find(group.name_) == manipulators.end())
      {
        std::shared_ptr<Chain_T> manip(new Chain_T());
        if (!manip->init(scene_graph, group.chains_.front().first, group.chains_.front().second, group.name_))
        {
          CONSOLE_BRIDGE_logError("Failed to create kinematic chaing for manipulator %s!", group.name_);
        }
        else
        {
          manipulators.insert(std::make_pair(group.name_, manip));
        }
      }
    }

    if (!group.joints_.empty())
    {
      if (manipulators.find(group.name_) == manipulators.end())
      {
        std::shared_ptr<Tree_T> manip(new Tree_T());
        if (!manip->init(scene_graph, group.joints_, group.name_))
        {
          CONSOLE_BRIDGE_logError("Failed to create kinematic chaing for manipulator %s!", group.name_);
        }
        else
        {
          manipulators.insert(std::make_pair(group.name_, manip));
        }
      }
    }

    // TODO: Need to add other options
    if (!group.links_.empty())
    {
      CONSOLE_BRIDGE_logError("Link groups are currently not supported!");
    }

    if (!group.subgroups_.empty())
    {
      CONSOLE_BRIDGE_logError("Subgroups are currently not supported!");
    }
  }

  return manipulators;
}

/**
 * @brief This will check if the provided joint values are within the privided limits
 * @param joint_values The joint values to check
 * @param limits The joint limits
 * @return True if joint values are within the joint limits, otherwise false
 */
template <typename FloatType>
inline static bool isWithinLimits(const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1> >& joint_values,
                                  const Eigen::MatrixX2d& limits)
{
  for (int i = 0; i < limits.rows(); ++i)
    if ((joint_values[i] < limits(i, 0)) || (joint_values[i] > limits(i, 1)))
      return false;

  return true;
}

/**
 * @brief Kinematics only return solution between PI and -PI. Provided the limits it will append redundant solutions.
 * @param sol The current solution returned from OPW kinematics
 * @param limits The joint limits of the robot
 */
template <typename FloatType>
inline std::vector<FloatType> getRedundantSolutions(const FloatType* sol, const Eigen::MatrixX2d& limits)
{
  auto dof = static_cast<int>(limits.rows());
  FloatType val;
  std::vector<FloatType> redundant_sols;
  for (int i = 0; i < dof; ++i)
  {
    val = sol[i];
    while ((val -= (2 * M_PI)) > limits(i, 0))
    {
      std::vector<FloatType> new_sol(sol, sol + dof);
      new_sol[static_cast<size_t>(i)] = val;
      redundant_sols.insert(redundant_sols.end(), new_sol.begin(), new_sol.end());
    }

    val = sol[i];
    while ((val += (static_cast<FloatType>(2.0 * M_PI))) < limits(i, 1))
    {
      std::vector<FloatType> new_sol(sol, sol + dof);
      new_sol[static_cast<size_t>(i)] = val;
      redundant_sols.insert(redundant_sols.end(), new_sol.begin(), new_sol.end());
    }
  }

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
inline bool isValid(const FloatType* qs, int dof)
{
  for (int i = 0; i < dof; ++i)
    if (!std::isfinite(qs[i]))
      return false;

  return true;
}

/**
 * @brief This take an array of floats and modifies them in place to be between [-PI, PI]
 * @param qs A pointer to a float array
 * @param dof The length of the float array
 */
template <typename FloatType>
inline void harmonizeTowardZero(FloatType* qs, int dof)
{
  const static auto pi = FloatType(M_PI);
  const static auto two_pi = FloatType(2.0 * M_PI);

  for (int i = 0; i < dof; i++)
  {
    if (qs[i] > pi)
      qs[i] -= two_pi;
    else if (qs[i] < -pi)
      qs[i] += two_pi;
  }
}

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_UTILS_H
