/**
 * @file ompl_planner_config.hpp
 * @brief Tesseract OMPL planner config implementation.
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIG_HPP
#define TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIG_HPP

#include <tesseract_motion_planners/ompl/config/ompl_planner_config.h>
#include <tesseract/tesseract.h>

namespace tesseract_motion_planners
{
template <typename PlannerType>
OMPLPlannerConfig<PlannerType>::OMPLPlannerConfig(tesseract::Tesseract::ConstPtr tesseract_, std::string manipulator_)
  : tesseract(std::move(tesseract_)), manipulator(std::move(manipulator_))
{
}

template <typename PlannerType>
OMPLPlannerConfig<PlannerType>::OMPLPlannerConfig(tesseract::Tesseract::ConstPtr tesseract_,
                                                  std::string manipulator_,
                                                  ompl::geometric::SimpleSetupPtr simple_setup)
  : simple_setup(std::move(simple_setup)), tesseract(std::move(tesseract_)), manipulator(std::move(manipulator_))
{
}

template <typename PlannerType>
bool OMPLPlannerConfig<PlannerType>::generate()
{
  return ((simple_setup != nullptr) && (tesseract != nullptr) && (!manipulator.empty()));
}

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIG_HPP
