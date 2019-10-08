/**
 * @file descartes_positioner_sampler.h
 * @brief Tesseract Descartes Robot + External Positioner Sampler
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_POSITIONER_SAMPLER_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_POSITIONER_SAMPLER_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/utils.h>
#include <Eigen/Dense>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_motion_planners/descartes/tool_pose_samplers.h>
#include <tesseract_motion_planners/descartes/types.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
class DescartesPositionerSampler : public descartes_light::PositionSampler<FloatType>
{
public:
  /**
   * @brief This is a descartes sampler for a robot + external positioner.
   *
   * This assumes the tool_pose is relative to the positioner tip link.
   *
   * @param tool_pose The tool center point applied to robot kinematics
   * @param tool_pose_sampler The tool pose sampler function to be used
   * @param positioner_kinematics The forward kinematics object for the external positioner system
   * @param robot_kinematics The robot inverse kinematics object
   * @param collision The collision interface
   * @param curret_state The currect state of the system
   * @param positioner_sample_resolution The positioner system sampling resolution
   * @param robot_tcp The robot tcp to be used.
   * @param robot_reach The reach of the robot. Used to filter positioner samples.
   * @param allow_collision If true and no valid solution was found it will return the best of the worst
   * @param is_valid This is a user defined function to filter out solution
   */
  DescartesPositionerSampler(const Eigen::Isometry3d tool_pose,
                             const tesseract_motion_planners::ToolPoseSamplerFn tool_pose_sampler,
                             const tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics,
                             const tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
                             const typename descartes_light::CollisionInterface<FloatType>::Ptr collision,
                             const tesseract_environment::EnvState::ConstPtr current_state,
                             const Eigen::VectorXd positioner_sample_resolution,
                             const Eigen::Isometry3d robot_tcp,
                             const double robot_reach,
                             const bool allow_collision,
                             const DescartesIsValidFn<FloatType> is_valid = nullptr);

  bool sample(std::vector<FloatType>& solution_set) override;

private:
  Eigen::Isometry3d tool_pose_;
  tesseract_motion_planners::ToolPoseSamplerFn tool_pose_sampler_;
  tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics_;
  tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics_;
  typename descartes_light::CollisionInterface<FloatType>::Ptr collision_;
  Eigen::Isometry3d world_to_positioner_base_;
  Eigen::Isometry3d world_to_robot_base_;
  Eigen::Matrix2d positioner_limits_;
  Eigen::VectorXd positioner_sample_resolution_;
  Eigen::Isometry3d robot_tcp_;
  FloatType robot_reach_;
  bool allow_collision_;
  int dof_;
  Eigen::VectorXd ik_seed_;
  DescartesIsValidFn<FloatType> is_valid_;

  bool isCollisionFree(const FloatType* vertex);

  bool getBestSolution(std::vector<FloatType>& solution_set,
                       const tesseract_common::VectorIsometry3d& tool_poses,
                       const std::vector<Eigen::VectorXd>& dof_range);

  void nested_ik(const int loop_level,
                 const std::vector<Eigen::VectorXd>& dof_range,
                 const Eigen::Isometry3d& p,
                 Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample_pose,
                 std::vector<FloatType>& solution_set,
                 const bool get_best_solution,
                 double& distance);

  bool ikAt(const Eigen::Isometry3d& p,
            const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& positioner_pose,
            std::vector<FloatType>& solution_set,
            const bool get_best_solution,
            double& distance);
};

using DescartesPositionerSamplerF = DescartesPositionerSampler<float>;
using DescartesPositionerSamplerD = DescartesPositionerSampler<double>;

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_POSITIONER_SAMPLER_H
