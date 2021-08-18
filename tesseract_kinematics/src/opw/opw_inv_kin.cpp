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
InverseKinematics::UPtr OPWInvKin::clone() const { return std::make_unique<OPWInvKin>(*this); }

// bool OPWInvKin::update()
//{
//  if (!init(name_,
//            params_,
//            base_link_name_,
//            tip_link_name_,
//            orig_data_.joint_names,
//            orig_data_.link_names,
//            orig_data_.active_link_names,
//            orig_data_.limits))
//    return false;

//  if (sync_fwd_kin_ != nullptr)
//    synchronize(sync_fwd_kin_);

//  return true;
//}

OPWInvKin::OPWInvKin(const OPWInvKin& other) { *this = other; }

OPWInvKin& OPWInvKin::operator=(const OPWInvKin& other)
{
  initialized_ = other.initialized_;
  name_ = other.name_;
  base_link_name_ = other.base_link_name_;
  tip_link_name_ = other.tip_link_name_;
  joint_names_ = other.joint_names_;
  params_ = other.params_;
  solver_name_ = other.solver_name_;
  return *this;
}

IKSolutions OPWInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                  const std::string& working_frame,
                                  const std::string& link_name,
                                  const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  assert(working_frame == base_link_name_);
  assert(link_name == tip_link_name_);

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
      solution_set.push_back(eigen_sol);
    }
  }

  return solution_set;
}

Eigen::Index OPWInvKin::numJoints() const { return 6; }

std::vector<std::string> OPWInvKin::getJointNames() const { return joint_names_; }
std::string OPWInvKin::getBaseLinkName() const { return base_link_name_; }
std::vector<std::string> OPWInvKin::getWorkingFrames() const { return { base_link_name_ }; }
std::vector<std::string> OPWInvKin::getTipLinkNames() const { return { tip_link_name_ }; }
std::string OPWInvKin::getName() const { return name_; }
std::string OPWInvKin::getSolverName() const { return solver_name_; }

bool OPWInvKin::init(std::string name,
                     opw_kinematics::Parameters<double> params,
                     std::string base_link_name,
                     std::string tip_link_name,
                     std::vector<std::string> joint_names)
{
  assert(joint_names.size() == 6);

  name_ = std::move(name);
  params_ = params;
  base_link_name_ = std::move(base_link_name);
  tip_link_name_ = std::move(tip_link_name);
  joint_names_ = std::move(joint_names);
  initialized_ = true;

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
