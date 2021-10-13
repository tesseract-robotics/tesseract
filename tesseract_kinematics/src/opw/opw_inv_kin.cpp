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
#include <opw_kinematics/opw_utilities.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
OPWInvKin::OPWInvKin(opw_kinematics::Parameters<double> params,
                     std::string base_link_name,
                     std::string tip_link_name,
                     std::vector<std::string> joint_names,
                     std::string solver_name)
  : params_(params)
  , base_link_name_(std::move(base_link_name))
  , tip_link_name_(std::move(tip_link_name))
  , joint_names_(std::move(joint_names))
  , solver_name_(std::move(solver_name))
{
  if (joint_names_.size() != 6)
    throw std::runtime_error("OPWInvKin, only support six joints!");
}

InverseKinematics::UPtr OPWInvKin::clone() const { return std::make_unique<OPWInvKin>(*this); }

OPWInvKin::OPWInvKin(const OPWInvKin& other) { *this = other; }

OPWInvKin& OPWInvKin::operator=(const OPWInvKin& other)
{
  base_link_name_ = other.base_link_name_;
  tip_link_name_ = other.tip_link_name_;
  joint_names_ = other.joint_names_;
  params_ = other.params_;
  solver_name_ = other.solver_name_;
  return *this;
}

IKSolutions OPWInvKin::calcInvKin(const tesseract_common::TransformMap& tip_link_poses,
                                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  assert(tip_link_poses.size() == 1);
  assert(tip_link_poses.find(tip_link_name_) != tip_link_poses.end());

  opw_kinematics::Solutions<double> sols = opw_kinematics::inverse(params_, tip_link_poses.at(tip_link_name_));

  // Check the output
  IKSolutions solution_set;
  solution_set.reserve(sols.size());
  for (auto& sol : sols)
  {
    if (opw_kinematics::isValid<double>(sol))
    {
      Eigen::Map<Eigen::VectorXd> eigen_sol(sol.data(), static_cast<Eigen::Index>(sol.size()));

      // Harmonize between [-PI, PI]
      harmonizeTowardZero<double>(eigen_sol);  // Modifies 'sol' in place

      // Add solution
      solution_set.push_back(eigen_sol);
    }
  }

  return solution_set;
}

Eigen::Index OPWInvKin::numJoints() const { return 6; }

std::vector<std::string> OPWInvKin::getJointNames() const { return joint_names_; }
std::string OPWInvKin::getBaseLinkName() const { return base_link_name_; }
std::string OPWInvKin::getWorkingFrame() const { return base_link_name_; }
std::vector<std::string> OPWInvKin::getTipLinkNames() const { return { tip_link_name_ }; }
std::string OPWInvKin::getSolverName() const { return solver_name_; }

}  // namespace tesseract_kinematics
