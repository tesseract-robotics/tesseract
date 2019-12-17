/**
 * @file descartes_robot_sampler.hpp
 * @brief Tesseract Descartes Kinematics Sampler Implementation
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_HPP

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/utils.h>
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
DescartesRobotSampler<FloatType>::DescartesRobotSampler(
    const Eigen::Isometry3d& target_pose,
    PoseSamplerFn target_pose_sampler,
    tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
    typename descartes_light::CollisionInterface<FloatType>::Ptr collision,
    const tesseract_environment::EnvState::ConstPtr& current_state,
    const Eigen::Isometry3d& robot_tcp,
    double robot_reach,
    bool allow_collision,
    DescartesIsValidFn<FloatType> is_valid)
  : target_pose_(target_pose)
  , target_pose_sampler_(std::move(target_pose_sampler))
  , robot_kinematics_(std::move(robot_kinematics))
  , collision_(std::move(collision))
  , world_to_robot_base_(current_state->transforms.at(robot_kinematics_->getBaseLinkName()))
  , robot_tcp_(robot_tcp)
  , robot_reach_(robot_reach)
  , allow_collision_(allow_collision)
  , dof_(static_cast<int>(robot_kinematics_->numJoints()))
  , ik_seed_(Eigen::VectorXd::Zero(dof_))
  , is_valid_(std::move(is_valid))
{
}

template <typename FloatType>
bool DescartesRobotSampler<FloatType>::sample(std::vector<FloatType>& solution_set)
{
  double distance = std::numeric_limits<double>::min();
  tesseract_common::VectorIsometry3d target_poses = target_pose_sampler_(target_pose_);
  for (const auto& sp : target_poses)
  {
    // Tool pose in rail coordinate system
    Eigen::Isometry3d target_pose = world_to_robot_base_.inverse() * sp * robot_tcp_.inverse();
    ikAt(solution_set, target_pose, false, distance);
  }

  if (solution_set.empty() && allow_collision_)
    getBestSolution(solution_set, target_poses);

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesRobotSampler<FloatType>::isCollisionFree(const FloatType* vertex)
{
  if (collision_ == nullptr)
    return true;

  return collision_->validate(vertex, static_cast<size_t>(dof_));
}

template <typename FloatType>
bool DescartesRobotSampler<FloatType>::ikAt(std::vector<FloatType>& solution_set,
                                            const Eigen::Isometry3d& target_pose,
                                            bool get_best_solution,
                                            double& distance)
{
  if (target_pose.translation().norm() > robot_reach_)
    return false;

  Eigen::VectorXd robot_solution_set;
  auto robot_dof = static_cast<int>(robot_kinematics_->numJoints());
  if (!robot_kinematics_->calcInvKin(robot_solution_set, target_pose, ik_seed_))
    return false;

  long num_sols = robot_solution_set.size() / robot_dof;
  for (long i = 0; i < num_sols; i++)
  {
    double* sol = robot_solution_set.data() + robot_dof * i;

    std::vector<FloatType> full_sol;
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
bool DescartesRobotSampler<FloatType>::getBestSolution(std::vector<FloatType>& solution_set,
                                                       const tesseract_common::VectorIsometry3d& target_poses)
{
  double distance = std::numeric_limits<double>::min();
  for (const auto& sp : target_poses)
  {
    // Tool pose in rail coordinate system
    Eigen::Isometry3d target_pose = world_to_robot_base_.inverse() * sp * robot_tcp_.inverse();
    ikAt(solution_set, target_pose, false, distance);
  }

  return !solution_set.empty();
}

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_HPP
