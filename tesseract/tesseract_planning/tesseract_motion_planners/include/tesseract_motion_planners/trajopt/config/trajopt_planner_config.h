/**
 * @file trajopt_planner_config.h
 * @brief A simple TrajOpt configuration class constructed from a TrajOpt problem
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_CONFIG_H

#include "tesseract_motion_planners/trajopt/config/trajopt_planner_config_base.h"

namespace tesseract_motion_planners
{
/**
 * @brief Simple planner configuration struct that takes a completed TrajOptProb directly
 * and saves it internally
 */
struct TrajOptPlannerConfig : public TrajOptPlannerConfigBase
{
  TrajOptPlannerConfig(trajopt::TrajOptProb::Ptr prob_);

  virtual bool generate() override;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_PLANNER_CONFIG_H
