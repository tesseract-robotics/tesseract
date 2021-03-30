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
InverseKinematics::Ptr OPWInvKin::clone() const
{
  auto cloned_invkin = std::make_shared<OPWInvKin>();
  cloned_invkin->init(*this);
  return cloned_invkin;
}

bool OPWInvKin::update()
{
  return init(name_, params_, base_link_name_, tip_link_name_, joint_names_, link_names_, active_link_names_, limits_);
}

IKSolutions OPWInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  opw_kinematics::Solutions<double> sols = opw_kinematics::inverse(params_, pose);

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
      if (tesseract_common::satisfiesPositionLimits(eigen_sol, limits_.joint_limits))
        solution_set.push_back(eigen_sol);

      // Add redundant solutions
      IKSolutions redundant_sols = getRedundantSolutions<double>(eigen_sol, limits_.joint_limits);
      if (!redundant_sols.empty())
      {
        solution_set.insert(end(solution_set),
                            std::make_move_iterator(redundant_sols.begin()),
                            std::make_move_iterator(redundant_sols.end()));
      }
    }
  }

  return solution_set;
}

IKSolutions OPWInvKin::calcInvKin(const Eigen::Isometry3d& /*pose*/,
                                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/,
                                  const std::string& /*link_name*/) const
{
  throw std::runtime_error("OPWInvKin::calcInvKin(const Eigen::Isometry3d&, const Eigen::Ref<const Eigen::VectorXd>&, "
                           "const std::string&) Not Supported!");
}

bool OPWInvKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != numJoints())
  {
    CONSOLE_BRIDGE_logError(
        "Number of joint angles (%d) don't match robot_model (%d)", static_cast<int>(vec.size()), numJoints());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < limits_.joint_limits(i, 0)) || (vec(i) > limits_.joint_limits(i, 1)))
    {
      CONSOLE_BRIDGE_logDebug("Joint %s is out-of-range (%g < %g < %g)",
                              joint_names_[static_cast<size_t>(i)].c_str(),
                              limits_.joint_limits(i, 0),
                              vec(i),
                              limits_.joint_limits(i, 1));
      return false;
    }
  }

  return true;
}

unsigned int OPWInvKin::numJoints() const { return 6; }

const std::vector<std::string>& OPWInvKin::getJointNames() const { return joint_names_; }
const std::vector<std::string>& OPWInvKin::getLinkNames() const { return link_names_; }
const std::vector<std::string>& OPWInvKin::getActiveLinkNames() const { return active_link_names_; }
const tesseract_common::KinematicLimits& OPWInvKin::getLimits() const { return limits_; }

void OPWInvKin::setLimits(tesseract_common::KinematicLimits limits)
{
  unsigned int nj = numJoints();
  if (limits.joint_limits.rows() != nj || limits.velocity_limits.size() != nj ||
      limits.acceleration_limits.size() != nj)
    throw std::runtime_error("Kinematics limits assigned are invalid!");

  limits_ = std::move(limits);
}

const std::string& OPWInvKin::getBaseLinkName() const { return base_link_name_; }
const std::string& OPWInvKin::getTipLinkName() const { return tip_link_name_; }
const std::string& OPWInvKin::getName() const { return name_; }
const std::string& OPWInvKin::getSolverName() const { return solver_name_; }

bool OPWInvKin::init(std::string name,
                     opw_kinematics::Parameters<double> params,
                     std::string base_link_name,
                     std::string tip_link_name,
                     std::vector<std::string> joint_names,
                     std::vector<std::string> link_names,
                     std::vector<std::string> active_link_names,
                     tesseract_common::KinematicLimits limits)
{
  assert(joint_names.size() == 6);

  name_ = std::move(name);
  params_ = params;
  base_link_name_ = std::move(base_link_name);
  tip_link_name_ = std::move(tip_link_name);
  joint_names_ = std::move(joint_names);
  link_names_ = std::move(link_names);
  active_link_names_ = std::move(active_link_names);
  limits_ = std::move(limits);
  initialized_ = true;

  return initialized_;
}

bool OPWInvKin::init(const OPWInvKin& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  params_ = kin.params_;
  solver_name_ = kin.solver_name_;
  base_link_name_ = kin.base_link_name_;
  tip_link_name_ = kin.tip_link_name_;
  joint_names_ = kin.joint_names_;
  link_names_ = kin.link_names_;
  active_link_names_ = kin.active_link_names_;
  limits_ = kin.limits_;

  return initialized_;
}

bool OPWInvKin::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics
