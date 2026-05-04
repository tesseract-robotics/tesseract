/**
 * @file utils.h
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
#ifndef TESSERACT_KINEMATICS_UTILS_H
#define TESSERACT_KINEMATICS_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <console_bridge/console.h>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/common/kinematic_limits.h>
#include <tesseract/scene_graph/fwd.h>

namespace tesseract::kinematics
{
template <typename FloatType>
using VectorX = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

class JointGroup;
class ForwardKinematics;

/**
 * @brief Numerically calculate a jacobian. This is mainly used for testing
 * @param jacobian (Return) The jacobian which gets filled out.
 * @param change_base  The transform from the desired frame to the current base frame of the jacobian
 * @param kin          The kinematics object
 * @param joint_values The joint values for which to calculate the jacobian
 * @param link_name    The link_name for which the jacobian should be calculated
 * @param link_point   The point on the link for which to calculate the jacobian
 */
void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                       const Eigen::Isometry3d& change_base,
                       const ForwardKinematics& kin,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                       const std::string& link_name,
                       const Eigen::Ref<const Eigen::Vector3d>& link_point);

/**
 * @brief Numerically calculate a jacobian. This is mainly used for testing
 * @param jacobian (Return) The jacobian which gets filled out.
 * @param change_base  The transform from the desired frame to the current base frame of the jacobian
 * @param joint_group  The joint group object
 * @param joint_values The joint values for which to calculate the jacobian
 * @param link_name    The link_name for which the jacobian should be calculated
 * @param link_point   The point on the link for which to calculate the jacobian
 */
void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                       const Eigen::Isometry3d& change_base,
                       const JointGroup& joint_group,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                       const std::string& link_name,
                       const Eigen::Ref<const Eigen::Vector3d>& link_point);

/**
 * @brief Numerically calculate a jacobian when both source and target are active links
 * @param jacobian (Return) The jacobian which gets filled out.
 * @param joint_group       The joint group object
 * @param joint_values      The joint values for which to calculate the jacobian
 * @param base_link_name    The link name for which the jacobian is calculated in
 * @param base_link_offset  The offset on the base link for which to calculate the jacobian in
 * @param link_name         The link name for which the jacobian is calculated for
 * @param link_offset       The offset on the link for which the jacobian is calcualted for
 */
void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                       const JointGroup& joint_group,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                       const std::string& base_link_name,
                       const Eigen::Isometry3d& base_link_offset,
                       const std::string& link_name,
                       const Eigen::Isometry3d& link_offset);

/**
 * @brief Solve equation Ax=b for x
 * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
 * @param A Input matrix (represents Jacobian)
 * @param b Input vector (represents desired pose)
 * @param x Output vector (represents joint values)
 * @return True if solver completes properly
 */
bool solvePInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
               const Eigen::Ref<const Eigen::VectorXd>& b,
               Eigen::Ref<Eigen::VectorXd> x);

/**
 * @brief Calculate Damped Pseudoinverse
 * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
 * @param A Input matrix (represents Jacobian)
 * @param P Output matrix (represents pseudoinverse of A)
 * @param eps Singular value threshold
 * @param lambda Damping factor
 * @return True if Pseudoinverse completes properly
 */
bool dampedPInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                Eigen::Ref<Eigen::MatrixXd> P,
                double eps = 0.011,
                double lambda = 0.01);

/**
 * @brief Check if the provided jacobian is near a singularity
 * @details This is keep separated from the forward kinematics because special consideration may need to be made
 * based on the kinematics arrangement.
 * @param jacobian The jacobian to check if near a singularity
 * @param threshold The threshold that all singular values must be greater than or equal to not be considered near a
 * singularity
 */
bool isNearSingularity(const Eigen::Ref<const Eigen::MatrixXd>& jacobian, double threshold = 0.01);

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
Manipulability calcManipulability(const Eigen::Ref<const Eigen::MatrixXd>& jacobian);

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
      ss << "Lower limit of joint " << *current_index << " is infinite; no redundant solutions will be generated\n";
      CONSOLE_BRIDGE_logWarn(ss.str().c_str());
    }
    else
    {
      val = sol[*current_index];
      while ((val -= (2.0 * M_PI)) > limits(*current_index, 0) ||
             tesseract::common::almostEqualRelativeAndAbs(val, limits(*current_index, 0)))
      {
        // It not guaranteed that the provided solution is within limits so this check is needed
        if (val < limits(*current_index, 1) ||
            tesseract::common::almostEqualRelativeAndAbs(val, limits(*current_index, 1)))
        {
          /** @brief Making this thread_local does not help */
          Eigen::VectorXd new_sol = sol;
          new_sol[*current_index] = val;

          if (tesseract::common::satisfiesLimits<double>(new_sol, limits))
          {
            tesseract::common::enforceLimits<double>(new_sol, limits);
            redundant_sols.push_back(new_sol.template cast<FloatType>());
          }

          getRedundantSolutionsHelper<FloatType>(redundant_sols, new_sol, limits, current_index + 1, end_index);
        }
      }
    }

    if (std::isinf(limits(*current_index, 1)))
    {
      std::stringstream ss;
      ss << "Upper limit of joint " << *current_index << " is infinite; no redundant solutions will be generated\n";
      CONSOLE_BRIDGE_logWarn(ss.str().c_str());
    }
    else
    {
      val = sol[*current_index];
      while ((val += (2.0 * M_PI)) < limits(*current_index, 1) ||
             tesseract::common::almostEqualRelativeAndAbs(val, limits(*current_index, 1)))
      {
        // It not guaranteed that the provided solution is within limits so this check is needed
        if (val > limits(*current_index, 0) ||
            tesseract::common::almostEqualRelativeAndAbs(val, limits(*current_index, 0)))
        {
          /** @brief Making this thread_local does not help */
          Eigen::VectorXd new_sol = sol;
          new_sol[*current_index] = val;

          if (tesseract::common::satisfiesLimits<double>(new_sol, limits))
          {
            tesseract::common::enforceLimits<double>(new_sol, limits);
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
 * @param solutions The object to populate with redundant solutions
 * @param sol The solution to calculate redundant solutions about
 * @param limits The joint limits of the robot
 * @param redundancy_capable_joints The indices of the redundancy capable joints
 */
template <typename FloatType>
inline void getRedundantSolutions(std::vector<VectorX<FloatType>>& solutions,
                                  const Eigen::Ref<const VectorX<FloatType>>& sol,
                                  const Eigen::MatrixX2d& limits,
                                  const std::vector<Eigen::Index>& redundancy_capable_joints)
{
  if (redundancy_capable_joints.empty())
    return;

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

  getRedundantSolutionsHelper<FloatType>(solutions,
                                         sol.template cast<double>(),
                                         limits,
                                         redundancy_capable_joints.begin(),
                                         redundancy_capable_joints.end());
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
  std::vector<VectorX<FloatType>> redundant_sols;
  getRedundantSolutions<FloatType>(redundant_sols, sol, limits, redundancy_capable_joints);
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
 * @brief This takes an array of floats and modifies them in place to be between [-PI, PI]
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
    const auto diff = std::fmod(qs[i] + pi, two_pi);
    qs[i] = (diff < 0) ? (diff + pi) : (diff - pi);
  }
}

/**
 * @brief This takes the array of floats and modifies them in place to be between [-PI, PI] relative to limits median
 * @param qs A pointer to a float array
 * @param position_limits The limits leveraged for calculating median
 */
template <typename FloatType>
inline void harmonizeTowardMedian(Eigen::Ref<VectorX<FloatType>> qs,
                                  const std::vector<Eigen::Index>& redundancy_capable_joints,
                                  const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 2>>& position_limits)
{
  const static auto pi = FloatType(M_PI);
  const static auto two_pi = FloatType(2.0 * M_PI);

  for (const auto& i : redundancy_capable_joints)
  {
    const auto mean = (position_limits(i, 0) + position_limits(i, 1)) / 2.0;
    const auto diff = std::fmod((qs[i] - mean) + pi, two_pi);
    const auto vv = (diff < 0) ? (diff + pi) : (diff - pi);
    qs[i] = (vv + mean);
  }
}

/**
 * @brief Compute an upper bound on the Cartesian distance from @p base_link_name to @p tip_link_name
 * across all valid joint configurations of the chain.
 *
 * @details Walks the shortest path from @p base_link_name to @p tip_link_name and sums, per joint along
 * the path:
 *   - parent_to_joint_origin_transform.translation().norm()  (fixed geometric offset)
 *   - max(|lower|, |upper|)                                  (only for PRISMATIC joints)
 *
 * Revolute, continuous, and fixed joints contribute zero beyond their offset - rotation does not
 * translate the tip in the chain's own frame. The sum upper-bounds T_base_to_tip.translation().norm()
 * by the triangle inequality.
 *
 * Intended use: sizing early-exit reach filters in compound IK solvers (RTP/REP/ROP) so the filter
 * never fires on genuinely reachable targets.
 *
 * @param scene_graph    Scene graph to query. Must contain both @p base_link_name and @p tip_link_name
 *                       and have a path from one to the other.
 * @param base_link_name Chain start.
 * @param tip_link_name  Chain end.
 * @return Upper bound in metres. Always > 0 if the chain contains at least one joint with a
 *         non-zero offset or a prismatic extension; returns 0 for a zero-length chain
 *         (@p base_link_name == @p tip_link_name).
 * @throws std::runtime_error if either link is missing from @p scene_graph, if no path exists
 *         between them, if any joint along the path is FLOATING / PLANAR (unbounded translation),
 *         or if a PRISMATIC joint along the path has no finite limits.
 */
double computeChainReachUpperBound(const tesseract::scene_graph::SceneGraph& scene_graph,
                                   const std::string& base_link_name,
                                   const std::string& tip_link_name);

/**
 * @brief Look up each named joint in @p scene_graph and return their position limits as a (N,2) matrix.
 * @details Column 0 is the lower limit, column 1 the upper. Used by compound IK solvers (RTP/REP/ROP)
 *          to derive a default sampling range from joint limits when the caller did not supply one.
 * @throws std::runtime_error if any joint name is missing from @p scene_graph or if any matched
 *         joint has a null `limits` member.
 */
Eigen::MatrixX2d gatherJointLimits(const tesseract::scene_graph::SceneGraph& scene_graph,
                                   const std::vector<std::string>& joint_names);

/**
 * @brief Build a per-joint sample grid by uniformly subdividing each row of @p range using @p resolution.
 * @details For each joint i, returns `LinSpaced(cnt, range(i,0), range(i,1))` where
 *          `cnt = ceil(|range(i,1) - range(i,0)| / resolution(i)) + 1`. The number of samples is
 *          chosen so the actual step never exceeds the requested resolution.
 *
 * Used by compound IK solvers (RTP/REP/ROP) to discretise the auxiliary positioner / tool chain.
 *
 * @param range      (N,2) matrix; column 0 is the lower bound, column 1 the upper, per joint.
 * @param resolution (N,) vector; per-joint maximum step size. Must be > 0 elementwise.
 * @return Vector of N Eigen::VectorXd, each containing the sample points for one joint.
 */
std::vector<Eigen::VectorXd> buildSampleGrid(const Eigen::MatrixX2d& range,
                                             const Eigen::VectorXd& resolution);

}  // namespace tesseract::kinematics
#endif  // TESSERACT_KINEMATICS_UTILS_H
