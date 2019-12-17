/**
 * @file descartes_robot_positioner_sampler.hpp
 * @brief Tesseract descartes robot on a positioner sampler
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_ROBOT_POSITIONER_SAMPLER_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_ROBOT_POSITIONER_SAMPLER_HPP

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/utils.h>
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_robot_positioner_sampler.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
DescartesRobotPositionerSampler<FloatType>::DescartesRobotPositionerSampler(
    const Eigen::Isometry3d& target_pose,
    PoseSamplerFn target_pose_sampler,
    tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics,
    tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
    typename descartes_light::CollisionInterface<FloatType>::Ptr collision,
    const tesseract_environment::EnvState::ConstPtr& current_state,
    Eigen::VectorXd positioner_sample_resolution,
    const Eigen::Isometry3d& robot_tcp,
    double robot_reach,
    bool allow_collision,
    DescartesIsValidFn<FloatType> is_valid)
  : target_pose_(target_pose)
  , target_pose_sampler_(std::move(target_pose_sampler))
  , positioner_kinematics_(std::move(positioner_kinematics))
  , robot_kinematics_(std::move(robot_kinematics))
  , collision_(std::move(collision))
  , world_to_positioner_base_(current_state->transforms.at(positioner_kinematics_->getBaseLinkName()))
  , positioner_limits_(positioner_kinematics_->getLimits())
  , positioner_sample_resolution_(std::move(positioner_sample_resolution))
  , robot_tcp_(robot_tcp)
  , robot_reach_(robot_reach)
  , allow_collision_(allow_collision)
  , dof_(static_cast<int>(positioner_kinematics_->numJoints() + robot_kinematics_->numJoints()))
  , ik_seed_(Eigen::VectorXd::Zero(dof_))
  , is_valid_(std::move(is_valid))
{
}

template <typename FloatType>
bool DescartesRobotPositionerSampler<FloatType>::sample(std::vector<FloatType>& solution_set)
{
  double distance = std::numeric_limits<double>::min();
  auto num_joints = static_cast<int>(positioner_kinematics_->numJoints());

  // For the kinematics object to be sampled we need to create the joint values at the sampling resolution
  // The sampled joints results are stored in dof_range[joint index] to be used by the nested_ik function
  std::vector<Eigen::VectorXd> dof_range;
  dof_range.reserve(static_cast<std::size_t>(num_joints));
  for (int dof = 0; dof < num_joints; ++dof)
  {
    // given the sampling resolution for the joint calculate the number of samples such that the resolution is not
    // exceeded.
    int cnt = static_cast<int>(std::ceil(std::abs(positioner_limits_(dof, 1) - positioner_limits_(dof, 0)) /
                                         positioner_sample_resolution_(dof))) +
              1;
    dof_range.push_back(Eigen::VectorXd::LinSpaced(cnt, positioner_limits_(dof, 0), positioner_limits_(dof, 1)));
  }

  tesseract_common::VectorIsometry3d target_poses = target_pose_sampler_(target_pose_);
  for (const auto& sp : target_poses)
  {
    // Tool pose in rail coordinate system
    Eigen::Isometry3d target_pose = world_to_positioner_base_.inverse() * sp * robot_tcp_.inverse();
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> positioner_pose(num_joints);
    nested_ik(solution_set, 0, dof_range, target_pose, positioner_pose, false, distance);
  }

  if (solution_set.empty() && allow_collision_)
    getBestSolution(solution_set, target_poses, dof_range);

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesRobotPositionerSampler<FloatType>::isCollisionFree(const FloatType* vertex)
{
  if (collision_ == nullptr)
    return true;

  return collision_->validate(vertex, static_cast<size_t>(dof_));
}

template <typename FloatType>
void DescartesRobotPositionerSampler<FloatType>::nested_ik(
    std::vector<FloatType>& solution_set,
    int loop_level,
    const std::vector<Eigen::VectorXd>& dof_range,
    const Eigen::Isometry3d& target_pose,
    Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample_pose,
    bool get_best_solution,
    double& distance)
{
  if (loop_level >= static_cast<int>(positioner_kinematics_->numJoints()))
  {
    ikAt(solution_set, target_pose, sample_pose, get_best_solution, distance);
    return;
  }

  for (long i = 0; i < static_cast<long>(dof_range[static_cast<std::size_t>(loop_level)].size()); ++i)
  {
    sample_pose(loop_level) = static_cast<float>(dof_range[static_cast<std::size_t>(loop_level)][i]);
    nested_ik(solution_set, loop_level + 1, dof_range, target_pose, sample_pose, get_best_solution, distance);
  }
}

template <typename FloatType>
bool DescartesRobotPositionerSampler<FloatType>::ikAt(
    std::vector<FloatType>& solution_set,
    const Eigen::Isometry3d& target_pose,
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& positioner_pose,
    bool get_best_solution,
    double& distance)
{
  Eigen::Isometry3d positioner_tf;
  if (!positioner_kinematics_->calcFwdKin(positioner_tf, positioner_pose.template cast<double>()))
    return false;

  Eigen::Isometry3d robot_target_pose = positioner_tf.inverse() * target_pose;
  if (robot_target_pose.translation().norm() > robot_reach_)
    return false;

  Eigen::VectorXd robot_solution_set;
  auto robot_dof = static_cast<int>(robot_kinematics_->numJoints());
  if (!robot_kinematics_->calcInvKin(robot_solution_set, robot_target_pose, ik_seed_))
    return false;

  long num_sols = robot_solution_set.size() / robot_dof;
  for (long i = 0; i < num_sols; i++)
  {
    double* sol = robot_solution_set.data() + robot_dof * i;

    std::vector<FloatType> full_sol;
    full_sol.insert(end(full_sol), positioner_pose.data(), positioner_pose.data() + positioner_pose.size());
    full_sol.insert(end(full_sol), std::make_move_iterator(sol), std::make_move_iterator(sol + robot_dof));

    if ((is_valid_ != nullptr) && !is_valid_(Eigen::Map<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>(
                                      full_sol.data(), static_cast<long>(full_sol.size()))))
      continue;

    if (!get_best_solution)
    {
      if (isCollisionFree(full_sol.data()))
        solution_set.insert(
            end(solution_set), std::make_move_iterator(full_sol.begin()), std::make_move_iterator(full_sol.end()));
    }
    else
    {
      double cur_distance = collision_->distance(full_sol.data(), full_sol.size());
      if (cur_distance > distance)
      {
        distance = cur_distance;
        solution_set.insert(
            begin(solution_set), std::make_move_iterator(full_sol.begin()), std::make_move_iterator(full_sol.end()));
      }
    }
  }

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesRobotPositionerSampler<FloatType>::getBestSolution(std::vector<FloatType>& solution_set,
                                                                 const tesseract_common::VectorIsometry3d& target_poses,
                                                                 const std::vector<Eigen::VectorXd>& dof_range)
{
  double distance = std::numeric_limits<double>::min();
  auto num_joints = static_cast<int>(positioner_kinematics_->numJoints());
  for (const auto& sp : target_poses)
  {
    // Tool pose in rail coordinate system
    Eigen::Isometry3d target_pose = world_to_positioner_base_.inverse() * sp * robot_tcp_.inverse();
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> positioner_pose(num_joints);
    nested_ik(solution_set, 0, dof_range, target_pose, positioner_pose, true, distance);
  }

  return !solution_set.empty();
}

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_ROBOT_POSITIONER_SAMPLER_HPP
