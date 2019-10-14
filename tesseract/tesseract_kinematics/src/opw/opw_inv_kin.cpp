/**
 * @file opw_inv_kin.cpp
 * @brief Tesseract OPW Inverse kinematics implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <console_bridge/console.h>
#include <opw_kinematics/opw_kinematics.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
OPWInvKin::OPWInvKin(const std::string name,
                     const opw_kinematics::Parameters<double> params,
                     const std::string base_link_name,
                     const std::string tip_link_name,
                     const std::vector<std::string> joint_names,
                     const std::vector<std::string> link_names,
                     const std::vector<std::string> active_link_names,
                     const Eigen::MatrixX2d joint_limits)
  : name_(name)
  , params_(std::move(params))
  , base_link_name_(std::move(base_link_name))
  , tip_link_name_(std::move(tip_link_name))
  , joint_names_(std::move(joint_names))
  , link_names_(std::move(link_names))
  , active_link_names_(std::move(active_link_names))
  , joint_limits_(std::move(joint_limits))
{
  assert(joint_names_.size() == 6);
}

bool OPWInvKin::calcInvKin(Eigen::VectorXd& solutions,
                           const Eigen::Isometry3d& pose,
                           const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  std::array<double, 6 * 8> sols;
  opw_kinematics::inverse(params_, pose, sols.data());

  // Check the output
  std::vector<double> solution_set;
  solution_set.reserve(sols.size());
  for (int i = 0; i < 8; i++)
  {
    double* sol = sols.data() + 6 * i;
    if (isValid<double>(sol, 6))
    {
      harmonizeTowardZero<double>(sol, 6);  // Modifies 'sol' in place

      // Add solution
      solution_set.insert(end(solution_set), sol, sol + 6);

      // Add redundant solutions
      std::vector<double> redundant_sols = getRedundantSolutions(sol, joint_limits_);
      if (!redundant_sols.empty())
      {
        int num_sol = redundant_sols.size() / 6;
        for (int s = 0; s < num_sol; ++s)
        {
          double* redundant_sol = redundant_sols.data() + 6 * s;
          solution_set.insert(
              end(solution_set), std::make_move_iterator(redundant_sol), std::make_move_iterator(redundant_sol + 6));
        }
      }
    }
  }

  solutions = Eigen::Map<Eigen::VectorXd>(solution_set.data(), solution_set.size());
  return !solution_set.empty();
}

bool OPWInvKin::calcInvKin(Eigen::VectorXd& solutions,
                           const Eigen::Isometry3d& pose,
                           const Eigen::Ref<const Eigen::VectorXd>& seed,
                           const std::string& link_name) const
{
  throw std::runtime_error("IKFastInvKin::calcInvKin(Eigen::VectorXd&, const Eigen::Isometry3d&, const "
                           "Eigen::Ref<const Eigen::VectorXd>&, const std::string&) Not Supported!");
}

bool OPWInvKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != numJoints())
  {
    CONSOLE_BRIDGE_logError(
        "Number of joint angles (%d) don't match robot_model (%d)", static_cast<int>(vec.size()), numJoints());
    return false;
  }

  if (!isWithinLimits<double>(vec, joint_limits_))
    return false;

  return true;
}

unsigned int OPWInvKin::numJoints() const { return 6; }

}  // namespace tesseract_kinematics
