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

#ifndef IKFAST_HAS_LIBRARY
#define IKFAST_HAS_LIBRARY
#define IKFAST_NO_MAIN
#endif

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
inline InverseKinematics::UPtr IKFastInvKin::clone() const { return std::make_unique<IKFastInvKin>(*this); }

inline IKFastInvKin::IKFastInvKin(const IKFastInvKin& other) { *this = other; }

inline IKFastInvKin& IKFastInvKin::operator=(const IKFastInvKin& other)
{
  initialized_ = other.initialized_;
  name_ = other.name_;
  base_link_name_ = other.base_link_name_;
  tip_link_name_ = other.tip_link_name_;
  joint_names_ = other.joint_names_;
  solver_name_ = other.solver_name_;

  return *this;
}

// inline bool IKFastInvKin::update()
//{
//  if (!init(name_,
//            base_link_name_,
//            tip_link_name_,
//            orig_data_.joint_names,
//            orig_data_.link_names,
//            orig_data_.active_link_names,
//            orig_data_.limits,
//            orig_data_.redundancy_indices))
//    return false;

//  if (sync_fwd_kin_ != nullptr)
//    synchronize(sync_fwd_kin_);

//  return true;
//}

inline IKSolutions IKFastInvKin::calcInvKin(const Eigen::Isometry3d& pose,
                                            const std::string& working_frame,
                                            const std::string& link_name,
                                            const Eigen::Ref<const Eigen::VectorXd>& /*seed*/) const
{
  assert(working_frame == base_link_name_);
  assert(link_name == tip_link_name_);

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
  const std::size_t n_sols = ikfast_solution_set.GetNumSolutions();
  const std::size_t ikfast_dof = numJoints();

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
  int num_sol = static_cast<int>(sols.size() / ikfast_dof);
  IKSolutions solution_set;
  solution_set.reserve(sols.size());
  for (int i = 0; i < num_sol; i++)
  {
    Eigen::Map<Eigen::VectorXd> eigen_sol(sols.data() + static_cast<Eigen::Index>(ikfast_dof) * i,
                                          static_cast<Eigen::Index>(ikfast_dof));
    if (eigen_sol.array().allFinite())
    {
      harmonizeTowardZero<double>(eigen_sol);  // Modifies 'sol' in place
      solution_set.push_back(eigen_sol);
    }
  }

  return solution_set;
}

inline Eigen::Index IKFastInvKin::numJoints() const { return static_cast<Eigen::Index>(GetNumJoints()); }

inline bool IKFastInvKin::init(std::string name,
                               std::string base_link_name,
                               std::string tip_link_name,
                               std::vector<std::string> joint_names)
{
  name_ = std::move(name);
  base_link_name_ = std::move(base_link_name);
  tip_link_name_ = std::move(tip_link_name);
  joint_names_ = std::move(joint_names);
  initialized_ = true;

  return initialized_;
}

inline std::vector<std::string> IKFastInvKin::getJointNames() const { return joint_names_; }
inline std::string IKFastInvKin::getBaseLinkName() const { return base_link_name_; }
inline std::vector<std::string> IKFastInvKin::getTipLinkNames() const { return { tip_link_name_ }; }
inline std::string IKFastInvKin::getName() const { return name_; }
inline std::string IKFastInvKin::getSolverName() const { return solver_name_; }

inline bool IKFastInvKin::checkInitialized() const
{
  if (!initialized_)
  {
    CONSOLE_BRIDGE_logError("Kinematics has not been initialized!");
  }

  return initialized_;
}

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_IMPL_IKFAST_INV_KIN_HPP
