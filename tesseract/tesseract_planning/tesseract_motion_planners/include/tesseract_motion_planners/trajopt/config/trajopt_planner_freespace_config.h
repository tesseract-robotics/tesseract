/**
 * @file trajopt_planner_freespace_config.h
 * @brief A TrajOpt configuration class specifically for freespace planning
 *
 * @author Michael Ripperger
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_FREESPACE_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_FREESPACE_CONFIG_H

#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>

namespace tesseract_motion_planners
{
/**
 * @brief Default configuration to setup TrajOpt planner for freespace motions.
 *
 * While there are many parameters that can be set, the majority of them have defaults that will be suitable for many
 * problems. These are always required: tesseract_, maninpulator_, link_, tcp_
 *
 */
struct TrajOptPlannerFreespaceConfig : public TrajOptPlannerDefaultConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using TrajOptPlannerDefaultConfig::TrajOptPlannerDefaultConfig;

  std::shared_ptr<trajopt::ProblemConstructionInfo> generatePCI() const override;

  /** @brief The total number of timesteps used in the freespace motion. Default: 20 */
  int num_steps = 20;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_FREESPACE_CONFIG_H
