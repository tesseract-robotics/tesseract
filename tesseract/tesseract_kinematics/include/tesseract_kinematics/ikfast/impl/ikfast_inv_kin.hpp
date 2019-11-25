/**
 * @file ikfast_inv_kin.hpp
 * @brief Tesseract IKFast Inverse kinematics Wrapper Implementation
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
#ifndef TESSERACT_KINEMATICS_IMPL_IKFAST_INV_KIN_HPP
#define TESSERACT_KINEMATICS_IMPL_IKFAST_INV_KIN_HPP

#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <stdexcept>
#include <console_bridge/console.h>
#include <tesseract_kinematics/ikfast/external/ikfast.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_kinematics
{
InverseKinematics::Ptr IKFastInvKin::clone() const
{
  auto cloned_invkin = std::make_shared<IKFastInvKin>();
  cloned_invkin->init(*this);
  return std::move(cloned_invkin);
}

bool IKFastInvKin::calcInvKin(Eigen::VectorXd& solutions,
                              const Eigen::Isometry3d& pose,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) const
{
  // Convert to ikfast data type
  Eigen::Transform<IkReal, 3, Eigen::Isometry> ikfast_tcp = pose.cast<IkReal>();

  // Decompose
  const Eigen::Matrix<IkReal, 3, 1> translation = ikfast_tcp.translation();

  // Note the row major ordering here: IkFast expects the matrix in r00, r01, r02, ..., r11, r12, r13
  // ordering
  const Eigen::Matrix<IkReal, 3, 3, Eigen::RowMajor> rotation = ikfast_tcp.rotation();

  // Call IK (TODO: Make a better solution list class? One that uses vector instead of list)
  ikfast::IkSolutionList<IkReal> ikfast_solution_set;
  ComputeIk(translation.data(), rotation.data(), nullptr, ikfast_solution_set);

  // Unpack the solutions into the output vector
  const auto n_sols = ikfast_solution_set.GetNumSolutions();
  int ikfast_dof = numJoints();

  std::vector<IkReal> ikfast_output;
  ikfast_output.resize(n_sols * ikfast_dof);

  for (std::size_t i = 0; i < n_sols; ++i)
  {
    // This actually walks the list EVERY time from the start of i.
    const auto& sol = ikfast_solution_set.GetSolution(i);
    auto* out = ikfast_output.data() + i * ikfast_dof;
    sol.GetSolution(out, nullptr);
  }

  std::vector<double> sols;
  sols.insert(end(sols), std::make_move_iterator(ikfast_output.begin()), std::make_move_iterator(ikfast_output.end()));

  // Check the output
  int num_sol = sols.size() / ikfast_dof;
  std::vector<double> solution_set;
  solution_set.reserve(sols.size());
  for (int i = 0; i < num_sol; i++)
  {
    double* sol = sols.data() + ikfast_dof * i;
    if (isValid<double>(sol, ikfast_dof))
    {
      harmonizeTowardZero<double>(sol, ikfast_dof);  // Modifies 'sol' in place

      // Add solution
      if (isWithinLimits<double>(Eigen::Map<Eigen::VectorXd>(sol, ikfast_dof), joint_limits_))
        solution_set.insert(end(solution_set), sol, sol + ikfast_dof);

      // Add redundant solutions
      std::vector<double> redundant_sols = getRedundantSolutions(sol, joint_limits_);
      if (!redundant_sols.empty())
      {
        int num_sol = redundant_sols.size() / ikfast_dof;
        for (int s = 0; s < num_sol; ++s)
        {
          double* redundant_sol = redundant_sols.data() + ikfast_dof * s;
          solution_set.insert(end(solution_set),
                              std::make_move_iterator(redundant_sol),
                              std::make_move_iterator(redundant_sol + ikfast_dof));
        }
      }
    }
  }

  solutions = Eigen::Map<Eigen::VectorXd>(solution_set.data(), solution_set.size());
  return !solution_set.empty();
}

bool IKFastInvKin::calcInvKin(Eigen::VectorXd& solutions,
                              const Eigen::Isometry3d& pose,
                              const Eigen::Ref<const Eigen::VectorXd>& seed,
                              const std::string& link_name) const
{
  throw std::runtime_error("IKFastInvKin::calcInvKin(Eigen::VectorXd&, const Eigen::Isometry3d&, const "
                           "Eigen::Ref<const Eigen::VectorXd>&, const std::string&) Not Supported!");
}

bool IKFastInvKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
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

unsigned int IKFastInvKin::numJoints() const { return GetNumJoints(); }

bool IKFastInvKin::init(const std::string name,
                        const std::string base_link_name,
                        const std::string tip_link_name,
                        const std::vector<std::string> joint_names,
                        const std::vector<std::string> link_names,
                        const std::vector<std::string> active_link_names,
                        const Eigen::MatrixX2d joint_limits)
{
  name_ = std::move(name);
  base_link_name_ = std::move(base_link_name);
  tip_link_name_ = std::move(tip_link_name);
  joint_names_ = std::move(joint_names);
  link_names_ = std::move(link_names);
  active_link_names_ = std::move(active_link_names);
  joint_limits_ = std::move(joint_limits);
  initialized_ = true;

  return initialized_;
}

bool IKFastInvKin::init(const IKFastInvKin& kin)
{
  initialized_ = kin.initialized_;
  name_ = kin.name_;
  solver_name_ = kin.solver_name_;
  base_link_name_ = kin.base_link_name_;
  tip_link_name_ = kin.tip_link_name_;
  joint_names_ = kin.joint_names_;
  link_names_ = kin.link_names_;
  active_link_names_ = kin.active_link_names_;
  joint_limits_ = kin.joint_limits_;

  return initialized_;
}

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_IMPL_IKFAST_INV_KIN_HPP
