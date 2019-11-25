/**
 * @file descartes_robot_sampler.h
 * @brief Tesseract Descartes Kinematics Sampler
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/utils.h>
#include <Eigen/Dense>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_motion_planners/descartes/pose_samplers.h>
#include <tesseract_motion_planners/descartes/types.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
class DescartesRobotSampler : public descartes_light::PositionSampler<FloatType>
{
public:
  /**
   * @brief This is a descartes sampler for a robot.
   * @param target_pose The target pose in world coordinates applied to robot kinematics
   * @param target_pose_sampler The target pose sampler function to be used
   * @param robot_kinematics The robot inverse kinematics object
   * @param collision The collision interface
   * @param curret_state The currect state of the system
   * @param robot_tcp The robot tcp to be used.
   * @param robot_reach The reach of the robot. Used to filter rail samples.
   * @param allow_collision If true and no valid solution was found it will return the best of the worst
   * @param is_valid This is a user defined function to filter out solution
   */
  DescartesRobotSampler(const Eigen::Isometry3d& target_pose,
                        tesseract_motion_planners::PoseSamplerFn target_pose_sampler,
                        tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
                        typename descartes_light::CollisionInterface<FloatType>::Ptr collision,
                        const tesseract_environment::EnvState::ConstPtr& current_state,
                        const Eigen::Isometry3d& robot_tcp,
                        double robot_reach,
                        bool allow_collision,
                        DescartesIsValidFn<FloatType> is_valid);

  bool sample(std::vector<FloatType>& solution_set) override;

private:
  Eigen::Isometry3d target_pose_;                                          /**< @brief The target pose to sample */
  tesseract_motion_planners::PoseSamplerFn target_pose_sampler_;           /**< @brief Target pose sampler function */
  tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics_;     /**< @brief The robot inverse kinematics */
  typename descartes_light::CollisionInterface<FloatType>::Ptr collision_; /**< @brief The collision interface */
  Eigen::Isometry3d world_to_robot_base_; /**< @brief The transform from world to the base of the robot */
  Eigen::Isometry3d robot_tcp_;           /**< @brief The robot tool center point */
  double robot_reach_;                    /**< @brief The reach of the robot used to quickly filter samples */
  bool allow_collision_;    /**< @brief If true and no valid solution was found it will return the best of the worst */
  int dof_;                 /**< @brief The number of joints in the robot */
  Eigen::VectorXd ik_seed_; /**< @brief The seed for inverse kinematics which is zeros */
  DescartesIsValidFn<FloatType> is_valid_; /**< @brief This is a user defined function to filter out solution */

  /**
   * @brief Check if a solution is passes collision test
   * @param vertex The joint solution to check
   * @return True if collision test passed, otherwise false
   */
  bool isCollisionFree(const FloatType* vertex);

  /**
   * @brief This will return the best of the worst solution
   * @param solution_set The solution to populate
   * @param target_poses The target pose to sample
   * @return True if a solution was found, otherwise false
   */
  bool getBestSolution(std::vector<FloatType>& solution_set, const tesseract_common::VectorIsometry3d& target_poses);

  /**
   * @brief Solve inverse kinematics for the target pose
   * @param target_pose The target pose to solve inverse kinematics
   * @param solution_set The soltuion to return
   * @param get_best_solution Enable if best solution should be returned
   * @param distance The current best distance
   * @return True if a solution was found, otherwise false
   */
  bool ikAt(std::vector<FloatType>& solution_set,
            const Eigen::Isometry3d& target_pose,
            bool get_best_solution,
            double& distance);
};

using DescartesRobotSamplerF = DescartesRobotSampler<float>;
using DescartesRobotSamplerD = DescartesRobotSampler<double>;

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_ROBOT_SAMPLER_H
