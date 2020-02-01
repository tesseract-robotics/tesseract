/**
 * @file ompl_planner_freespace_config.h
 * @brief Tesseract OMPL planner freespace config.
 *
 * @author Levi Armstrong
 * @date January 22, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_FREESPACE_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_FREESPACE_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/config/ompl_planner_config.h>
#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{
using OptimizationObjectiveAllocator =
    std::function<ompl::base::OptimizationObjectivePtr(const ompl::base::SpaceInformationPtr&)>;

struct OMPLPlannerFreespaceConfig : public OMPLPlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<OMPLPlannerFreespaceConfig>;
  using ConstPtr = std::shared_ptr<const OMPLPlannerFreespaceConfig>;

  OMPLPlannerFreespaceConfig(tesseract::Tesseract::ConstPtr tesseract,
                             std::string manipulator,
                             std::vector<OMPLPlannerConfigurator::ConstPtr> planners);
  /** @brief Generates the OMPL problem and saves the result internally */
  bool generate() override;

  /**
   * @brief Determines the constraint placed at the start of the trajectory
   *
   * The planner currently only support joint waypoints. Need to expose the ability
   * to create a state sampler.
   */
  Waypoint::Ptr start_waypoint;

  /**
   * @brief Determines the constraint placed at the end of the trajectory
   *
   * The planner currently only support joint waypoints. Need to expose the ability
   * to create a state sampler.
   */
  Waypoint::Ptr end_waypoint;

  /** @brief This scales the variables search space. Must be same size as number of joints.
   *         If empty it defaults to all ones */
  Eigen::VectorXd weights;

  /** @brief The state sampler allocator. This can be null and it will use OMPL default state sampler allocator. */
  ompl::base::StateSamplerAllocator state_sampler_allocator =
      std::bind(&OMPLPlannerFreespaceConfig::allocWeightedRealVectorStateSampler, this, std::placeholders::_1);

  /** @brief Set the optimization objective function allocator. Default is to minimize path length */
  OptimizationObjectiveAllocator optimization_objective_allocator = [](const ompl::base::SpaceInformationPtr& si) {
    return std::make_shared<ompl::base::PathLengthOptimizationObjective>(si);
  };

  /** @brief The ompl state validity checker. If nullptr it uses OMPLFreespacePlanner::isStateValid. */
  ompl::base::StateValidityCheckerFn svc;

  /** @brief The ompl motion validator. If nullptr and continuous collision checking enabled it used
   * ContinuousMotionValidator */
  ompl::base::MotionValidatorPtr mv;

  /**
   * @brief Default State sampler which uses the weights information to scale the sampled state. This is use full
   * when you state space has mixed units like meters and radian.
   * @param space The ompl state space.
   * @return OMPL state sampler shared pointer
   */
  ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space) const;
};
}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_FREESPACE_CONFIG_H
