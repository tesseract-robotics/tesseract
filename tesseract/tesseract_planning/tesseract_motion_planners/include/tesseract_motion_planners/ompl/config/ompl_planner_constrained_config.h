/**
 * @file ompl_planner_constrained_config.h
 * @brief Tesseract OMPL planner constrained config.
 *
 * @author Levi Armstrong
 * @date February 24, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMP_PLANNER_CONSTRAINED_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_OMP_PLANNER_CONSTRAINED_CONFIG_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/Constraint.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_motion_planners
{
struct OMPLPlannerConstrainedConfig : public OMPLPlannerFreespaceConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<OMPLPlannerConstrainedConfig>;
  using ConstPtr = std::shared_ptr<const OMPLPlannerConstrainedConfig>;

  OMPLPlannerConstrainedConfig(tesseract::Tesseract::ConstPtr tesseract, std::string manipulator);

  OMPLPlannerConstrainedConfig(tesseract::Tesseract::ConstPtr tesseract,
                               std::string manipulator,
                               std::vector<OMPLPlannerConfigurator::ConstPtr> planners);
  /** @brief Generates the OMPL problem and saves the result internally */
  bool generate() override;

  /**
   * @brief The constraints on the problem
   *
   * When using constraints the set number of output state may not be achieved.
   */
  ompl::base::ConstraintPtr constraint{ nullptr };
};
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMP_PLANNER_CONSTRAINED_CONFIG_H
