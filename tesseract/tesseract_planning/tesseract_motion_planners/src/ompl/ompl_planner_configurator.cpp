/**
 * @file ompl_planner_configurator.cpp
 * @brief Tesseract OMPL planner configurators
 *
 * @author Levi Armstrong
 * @date February 1, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>

namespace tesseract_planning
{
ompl::base::PlannerPtr SBLConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::SBL>(si);
  planner->setRange(range);
  return planner;
}

ompl::base::PlannerPtr ESTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::EST>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  return planner;
}

ompl::base::PlannerPtr LBKPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::LBKPIECE1>(si);
  planner->setRange(range);
  planner->setBorderFraction(border_fraction);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

ompl::base::PlannerPtr BKPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::BKPIECE1>(si);
  planner->setRange(range);
  planner->setBorderFraction(border_fraction);
  planner->setFailedExpansionCellScoreFactor(failed_expansion_score_factor);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

ompl::base::PlannerPtr KPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::KPIECE1>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setBorderFraction(border_fraction);
  planner->setFailedExpansionCellScoreFactor(failed_expansion_score_factor);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

ompl::base::PlannerPtr RRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRT>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  return planner;
}

ompl::base::PlannerPtr RRTConnectConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRTConnect>(si);
  planner->setRange(range);
  return planner;
}

ompl::base::PlannerPtr RRTstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRTstar>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setDelayCC(delay_collision_checking);
  return planner;
}

ompl::base::PlannerPtr TRRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::TRRT>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setTempChangeFactor(temp_change_factor);
  planner->setInitTemperature(init_temperature);
  planner->setFrontierThreshold(frontier_threshold);
  planner->setFrontierNodeRatio(frontier_node_ratio);
  return planner;
}

ompl::base::PlannerPtr PRMConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::PRM>(si);
  planner->setMaxNearestNeighbors(static_cast<unsigned>(max_nearest_neighbors));
  return planner;
}

ompl::base::PlannerPtr PRMstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  return std::make_shared<ompl::geometric::PRMstar>(si);
}

ompl::base::PlannerPtr LazyPRMstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  return std::make_shared<ompl::geometric::LazyPRMstar>(si);
}

ompl::base::PlannerPtr SPARSConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::SPARS>(si);
  planner->setMaxFailures(static_cast<unsigned>(max_failures));
  planner->setDenseDeltaFraction(dense_delta_fraction);
  planner->setSparseDeltaFraction(sparse_delta_fraction);
  planner->setStretchFactor(stretch_factor);
  return planner;
}
}  // namespace tesseract_motion_planners
