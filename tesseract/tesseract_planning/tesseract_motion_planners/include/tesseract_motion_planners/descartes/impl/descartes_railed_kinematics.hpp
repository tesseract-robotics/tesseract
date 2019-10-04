/**
 * @file descartes_railed_kinematics.hpp
 * @brief Tesseract Descartes Railed Kinematics Implementation
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_RAILED_KINEMATICS_SAMPLER_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_RAILED_KINEMATICS_SAMPLER_HPP

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/utils.h>
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_railed_kinematics.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
DescartesRailedKinematics<FloatType>::DescartesRailedKinematics(
    const tesseract_kinematics::ForwardKinematics::ConstPtr railed_kinematics,
    const typename descartes_light::KinematicsInterface<FloatType>::ConstPtr robot_kinematics,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_railed_base,
    const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& railed_sample_resolution,
    const FloatType robot_reach)
  : railed_kinematics_(std::move(railed_kinematics))
  , robot_kinematics_(std::move(robot_kinematics))
  , world_to_railed_base_(world_to_railed_base)
  , railed_limits_(railed_kinematics->getLimits())
  , railed_sample_resolution_(railed_sample_resolution)
  , robot_reach_(robot_reach)
{
}

template <typename FloatType>
void DescartesRailedKinematics<FloatType>::nested_ik(
    const int loop_level,
    const std::vector<Eigen::VectorXd>& dof_range,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
    Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample_pose,
    std::vector<FloatType>& solution_set) const
{
  if (loop_level >= railed_kinematics_->numJoints())
  {
    ikAt(p, sample_pose, solution_set);
    return;
  }

  for (long i = 0; i < static_cast<long>(dof_range[static_cast<std::size_t>(loop_level)].size()); ++i)
  {
    sample_pose(loop_level) = dof_range[static_cast<std::size_t>(loop_level)][i];
    nested_ik(loop_level + 1, dof_range, p, sample_pose, solution_set);
  }
}

template <typename FloatType>
bool DescartesRailedKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                              std::vector<FloatType>& solution_set) const
{
  // Tool pose in rail coordinate system
  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose = world_to_railed_base_.inverse() * p;

  int num_joints = static_cast<int>(railed_kinematics_->numJoints());
  std::vector<Eigen::VectorXd> dof_range;
  dof_range.reserve(static_cast<std::size_t>(num_joints));
  for (int dof = 0; dof < num_joints; ++dof)
  {
    int cnt = std::ceil(std::abs(railed_limits_(dof, 1) - railed_limits_(dof, 0)) / railed_sample_resolution_(dof));
    dof_range.push_back(Eigen::VectorXd::LinSpaced(cnt, railed_limits_(dof, 0), railed_limits_(dof, 1)));
  }

  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> railed_pose(num_joints);
  nested_ik(0, dof_range, tool_pose, railed_pose, solution_set);

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesRailedKinematics<FloatType>::ikAt(
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& railed_pose,
    std::vector<FloatType>& solution_set) const
{
  Eigen::Isometry3d rail_tf;
  if (!railed_kinematics_->calcFwdKin(rail_tf, railed_pose.template cast<double>()))
    return false;

  Eigen::Transform<FloatType, 3, Eigen::Isometry> robot_tool_pose = (rail_tf.cast<FloatType>()).inverse() * p;
  if (robot_tool_pose.translation().norm() > robot_reach_)
    return false;

  std::vector<FloatType> robot_solution_set;
  int robot_dof = robot_kinematics_->dof();
  if (!robot_kinematics_->ik(robot_tool_pose, robot_solution_set))
    return false;

  long num_sols = static_cast<long>(robot_solution_set.size()) / robot_dof;
  for (long i = 0; i < num_sols; i++)
  {
    FloatType* sol = robot_solution_set.data() + robot_dof * i;
    solution_set.insert(end(solution_set),
                        railed_pose.data(),
                        railed_pose.data() + railed_pose.size());  // Insert the X-Y pose of the rail
    solution_set.insert(end(solution_set), sol, sol + robot_dof);  // And then insert the robot arm configuration
  }

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesRailedKinematics<FloatType>::fkAt(
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& railed_pose,
    const std::vector<FloatType>& pose,
    Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  if (!robot_kinematics_->fk(pose.data(), solution))
    return false;

  Eigen::Isometry3d railed_tf = Eigen::Isometry3d::Identity();
  if (!railed_kinematics_->calcFwdKin(railed_tf, railed_pose.template cast<double>()))
    return false;

  solution = world_to_railed_base_ * railed_tf.cast<FloatType>() * solution;
  return true;
}

template <typename FloatType>
bool DescartesRailedKinematics<FloatType>::fk(const FloatType* pose,
                                              Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  Eigen::Map<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> railed_pose(pose, railed_kinematics_->numJoints());
  std::vector<FloatType> robot_pose(pose + railed_kinematics_->numJoints(), pose + dof());

  return fkAt(railed_pose, robot_pose, solution);
}

template <typename FloatType>
int DescartesRailedKinematics<FloatType>::dof() const
{
  return static_cast<int>(railed_kinematics_->numJoints()) + robot_kinematics_->dof();
}

template <typename FloatType>
void DescartesRailedKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
{
  //  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "AnalyzeIK: ",
  //  ";"); std::stringstream ss; ss << p.matrix().format(CommaInitFmt); CONSOLE_BRIDGE_logInform(ss.str().c_str());

  //  // Tool pose in rail coordinate system
  //  Eigen::Transform<FloatType, 3, Eigen::Isometry> tool_pose = world_to_rail_base_.inverse() * p;

  //  const Eigen::Matrix<FloatType, 2, 1>& rail_lower_limit = rail_limits_.col(0);
  //  const Eigen::Matrix<FloatType, 2, 1>& rail_upper_limit = rail_limits_.col(1);

  //  const Eigen::Matrix<FloatType, 2, 1> origin(tool_pose.translation().x() -
  //  rail_base_to_robot_base_.translation().x(),
  //                                              tool_pose.translation().y() -
  //                                              rail_base_to_robot_base_.translation().y());

  //  const Eigen::Matrix<FloatType, 2, 1> x_range = getRange(origin.x(), rail_lower_limit.x(), rail_upper_limit.x());
  //  const Eigen::Matrix<FloatType, 2, 1> y_range = getRange(origin.y(), rail_lower_limit.y(), rail_upper_limit.y());

  //  const FloatType res_x =
  //      (x_range[1] - x_range[0]) / std::ceil((x_range[1] - x_range[0]) / rail_sample_resolution_.x());
  //  const FloatType res_y =
  //      (y_range[1] - y_range[0]) / std::ceil((y_range[1] - y_range[0]) / rail_sample_resolution_.y());

  //  for (FloatType x = x_range[0]; x < x_range[1]; x += res_x)
  //  {
  //    for (FloatType y = y_range[0]; y < y_range[1]; y += res_y)
  //    {
  //      const Eigen::Transform<FloatType, 3, Eigen::Isometry> world_to_robot_base =
  //          world_to_rail_base_ * Eigen::Translation<FloatType, 3>(x, y, static_cast<FloatType>(0.0)) *
  //          rail_base_to_robot_base_;
  //      const Eigen::Transform<FloatType, 3, Eigen::Isometry> in_robot = world_to_robot_base.inverse() * p;

  //      robot_kinematics_->analyzeIK(in_robot);
  //    }
  //  }
}

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_RAILED_KINEMATICS_SAMPLER_HPP
