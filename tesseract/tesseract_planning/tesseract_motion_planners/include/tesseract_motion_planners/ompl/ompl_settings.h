/**
 * @file ompl_settings.h
 * @brief Tesseract OMPL setting for planner.
 *
 * If a settings class does not exist for a planner available
 * in ompl you may simply create your own that has an apply
 * method that takes the specific planner you would like to use
 * and construct the OMPLFreespacePlanner with the desired planner
 * and newly created config class and everything should work.
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_SETTINGS_H
#define TESSERACT_MOTION_PLANNERS_OMPL_SETTINGS_H

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
#include <tesseract_motion_planners/ompl/ompl_settings.h>

namespace tesseract_motion_planners
{
template <typename PlannerT>
struct OMPLSettings
{
  void apply(PlannerT& planner) const;
};

template <>
struct OMPLSettings<ompl::geometric::SBL>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::SBL& planner) const { planner.setRange(range); }
};

template <>
struct OMPLSettings<ompl::geometric::EST>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::EST& planner) const
  {
    planner.setRange(range);
    planner.setGoalBias(goal_bias);
  }
};

template <>
struct OMPLSettings<ompl::geometric::LBKPIECE1>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Fraction of time focused on boarder (0.0,1.] */
  double border_fraction = 0.9;

  /** @brief Accept partially valid moves above fraction. */
  double min_valid_path_fraction = 0.5;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::LBKPIECE1& planner) const
  {
    planner.setRange(range);
    planner.setBorderFraction(border_fraction);
    planner.setMinValidPathFraction(min_valid_path_fraction);
  }
};

template <>
struct OMPLSettings<ompl::geometric::BKPIECE1>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Fraction of time focused on boarder (0.0,1.] */
  double border_fraction = 0.9;

  /** @brief When extending motion fails, scale score by factor. */
  double failed_expansion_score_factor = 0.5;

  /** @brief Accept partially valid moves above fraction. */
  double min_valid_path_fraction = 0.5;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::BKPIECE1& planner) const
  {
    planner.setRange(range);
    planner.setBorderFraction(border_fraction);
    planner.setFailedExpansionCellScoreFactor(failed_expansion_score_factor);
    planner.setMinValidPathFraction(min_valid_path_fraction);
  }
};

template <>
struct OMPLSettings<ompl::geometric::KPIECE1>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Fraction of time focused on boarder (0.0,1.] */
  double border_fraction = 0.9;

  /** @brief When extending motion fails, scale score by factor. */
  double failed_expansion_score_factor = 0.5;

  /** @brief Accept partially valid moves above fraction. */
  double min_valid_path_fraction = 0.5;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::KPIECE1& planner) const
  {
    planner.setRange(range);
    planner.setGoalBias(goal_bias);
    planner.setBorderFraction(border_fraction);
    planner.setFailedExpansionCellScoreFactor(failed_expansion_score_factor);
    planner.setMinValidPathFraction(min_valid_path_fraction);
  }
};

template <>
struct OMPLSettings<ompl::geometric::RRT>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::RRT& planner) const
  {
    planner.setRange(range);
    planner.setGoalBias(goal_bias);
  }
};

template <>
struct OMPLSettings<ompl::geometric::RRTConnect>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::RRTConnect& planner) const { planner.setRange(range); }
};

template <>
struct OMPLSettings<ompl::geometric::RRTstar>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Stop collision checking as soon as C-free parent found. */
  bool delay_collision_checking = true;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::RRTstar& planner) const
  {
    planner.setRange(range);
    planner.setGoalBias(goal_bias);
    planner.setDelayCC(delay_collision_checking);
  }
};

template <>
struct OMPLSettings<ompl::geometric::TRRT>
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief How much to increase or decrease temp. */
  double temp_change_factor = 2.0;

  /** @brief Initial temperature. */
  double init_temperature = 10e-6;

  /** @brief Dist new state to nearest neighbor to disqualify as frontier. */
  double frontier_threshold = 0.0;

  /** @brief 1/10, or 1 nonfrontier for every 10 frontier. */
  double frontier_node_ratio = 0.1;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::TRRT& planner) const
  {
    planner.setRange(range);
    planner.setGoalBias(goal_bias);
    planner.setTempChangeFactor(temp_change_factor);
    planner.setInitTemperature(init_temperature);
    planner.setFrontierThreshold(frontier_threshold);
    planner.setFrontierNodeRatio(frontier_node_ratio);
  }
};

template <>
struct OMPLSettings<ompl::geometric::PRM>
{
  /** @brief Use k nearest neighbors. */
  int max_nearest_neighbors = 10;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::PRM& planner) const
  {
    planner.setMaxNearestNeighbors(static_cast<unsigned>(max_nearest_neighbors));
  }
};

template <>
struct OMPLSettings<ompl::geometric::PRMstar>
{
  /** @brief Apply settings to planner */
  void apply(ompl::geometric::PRMstar& /*planner*/) const {}
};

template <>
struct OMPLSettings<ompl::geometric::LazyPRMstar>
{
  /** @brief Apply settings to planner */
  void apply(ompl::geometric::LazyPRMstar& /*planner*/) const {}
};

template <>
struct OMPLSettings<ompl::geometric::SPARS>
{
public:
  /** @brief The maximum number of failures before terminating the algorithm */
  int max_failures = 1000;

  /** @brief Dense graph connection distance as a fraction of max. extent */
  double dense_delta_fraction = 0.001;

  /** @brief Sparse Roadmap connection distance as a fraction of max. extent */
  double sparse_delta_fraction = 0.25;

  /** @brief The stretch factor in terms of graph spanners for SPARS to check against */
  double stretch_factor = 3;

  /** @brief Apply settings to planner */
  void apply(ompl::geometric::SPARS& planner) const
  {
    planner.setMaxFailures(static_cast<unsigned>(max_failures));
    planner.setDenseDeltaFraction(dense_delta_fraction);
    planner.setSparseDeltaFraction(sparse_delta_fraction);
    planner.setStretchFactor(stretch_factor);
  }
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_SETTINGS_H
