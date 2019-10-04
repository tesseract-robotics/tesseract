/**
 * @file descartes_positioner_kinematics.hpp
 * @brief Tesseract Descartes Robot + External Positioner Kinematics Implementation
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_POSITIONER_KINEMATICS_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_POSITIONER_KINEMATICS_HPP

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/utils.h>
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_positioner_kinematics.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
DescartesPositionerKinematics<FloatType>::DescartesPositionerKinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics,
    const typename descartes_light::KinematicsInterface<FloatType>::ConstPtr robot_kinematics,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_positioner_base,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& world_to_robot_base,
    const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& positioner_sample_resolution,
    const std::string point_link_name,
    const FloatType robot_reach,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry> positioner_point_link_name_transform)
  : positioner_kinematics_(std::move(positioner_kinematics))
  , positioner_point_link_name_transform_(std::move(positioner_point_link_name_transform))
  , robot_kinematics_(std::move(robot_kinematics))
  , world_to_positioner_base_(world_to_positioner_base)
  , world_to_robot_base_(world_to_robot_base)
  , positioner_limits_(positioner_kinematics->getLimits())
  , positioner_sample_resolution_(positioner_sample_resolution)
  , point_link_name_(std::move(point_link_name))
  , robot_reach_(robot_reach)
{
}

template <typename FloatType>
void DescartesPositionerKinematics<FloatType>::nested_ik(
    const int loop_level,
    const std::vector<Eigen::VectorXd>& dof_range,
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
    Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample_pose,
    std::vector<FloatType>& solution_set) const
{
  if (loop_level >= positioner_kinematics_->numJoints())
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
bool DescartesPositionerKinematics<FloatType>::ik(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
                                                  std::vector<FloatType>& solution_set) const
{
  // Tool pose in rail coordinate system

  Eigen::Transform<FloatType, 3, Eigen::Isometry> positioner_tool_pose = positioner_point_link_name_transform_ * p;

  int num_joints = static_cast<int>(positioner_kinematics_->numJoints());
  std::vector<Eigen::VectorXd> dof_range;
  dof_range.reserve(static_cast<std::size_t>(num_joints));
  for (int dof = 0; dof < num_joints; ++dof)
  {
    int cnt = std::ceil(std::abs(positioner_limits_(dof, 1) - positioner_limits_(dof, 0)) / positioner_sample_resolution_(dof));
    dof_range.push_back(Eigen::VectorXd::LinSpaced(cnt, positioner_limits_(dof, 0), positioner_limits_(dof, 1)));
  }

  Eigen::Matrix<FloatType, Eigen::Dynamic, 1> positioner_pose(num_joints);
  nested_ik(0, dof_range, positioner_tool_pose, positioner_pose, solution_set);

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesPositionerKinematics<FloatType>::ikAt(
    const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p,
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& positioner_pose,
    std::vector<FloatType>& solution_set) const
{
  Eigen::Isometry3d positioner_tf;
  if (!positioner_kinematics_->calcFwdKin(positioner_tf, positioner_pose.template cast<double>()))
    return false;

  Eigen::Transform<FloatType, 3, Eigen::Isometry> robot_tool_pose = world_to_robot_base_.inverse() * (world_to_positioner_base_ * positioner_tf.cast<FloatType>() * p);
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
                        positioner_pose.data(),
                        positioner_pose.data() + positioner_pose.size());  // Insert the X-Y pose of the rail
    solution_set.insert(end(solution_set), sol, sol + robot_dof);  // And then insert the robot arm configuration
  }

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesPositionerKinematics<FloatType>::fkAt(
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& positioner_pose,
    const std::vector<FloatType>& pose,
    Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  assert(false);
  return false;
}

template <typename FloatType>
bool DescartesPositionerKinematics<FloatType>::fk(const FloatType* pose,
                                                  Eigen::Transform<FloatType, 3, Eigen::Isometry>& solution) const
{
  assert(false);
  return false;
}

template <typename FloatType>
int DescartesPositionerKinematics<FloatType>::dof() const
{
  return static_cast<int>(positioner_kinematics_->numJoints()) + robot_kinematics_->dof();
}

template <typename FloatType>
void DescartesPositionerKinematics<FloatType>::analyzeIK(const Eigen::Transform<FloatType, 3, Eigen::Isometry>& p) const
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

#endif // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_POSITIONER_KINEMATICS_HPP
