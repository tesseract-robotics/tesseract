/**
 * @file ompl_planner_configurator.h
 * @brief Tesseract OMPL planner configurator.
 *
 * If a settings class does not exist for a planner available
 * in ompl you may simply create your own that has an apply
 * method that takes the specific planner you would like to use
 * and construct the OMPLFreespacePlanner with the desired planner
 * and newly created config class and everything should work.
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIGURATOR_H
#define TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIGURATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/Planner.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
struct OMPLPlannerConfigurator
{
  using Ptr = std::shared_ptr<OMPLPlannerConfigurator>;
  using ConstPtr = std::shared_ptr<const OMPLPlannerConfigurator>;

  OMPLPlannerConfigurator() = default;
  virtual ~OMPLPlannerConfigurator() = default;
  OMPLPlannerConfigurator(const OMPLPlannerConfigurator&) = default;
  OMPLPlannerConfigurator& operator=(const OMPLPlannerConfigurator&) = default;
  OMPLPlannerConfigurator(OMPLPlannerConfigurator&&) = default;
  OMPLPlannerConfigurator& operator=(OMPLPlannerConfigurator&&) = default;

  virtual ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const = 0;
};

struct SBLConfigurator : public OMPLPlannerConfigurator
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct ESTConfigurator : public OMPLPlannerConfigurator
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct LBKPIECE1Configurator : public OMPLPlannerConfigurator
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Fraction of time focused on boarder (0.0,1.] */
  double border_fraction = 0.9;

  /** @brief Accept partially valid moves above fraction. */
  double min_valid_path_fraction = 0.5;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct BKPIECE1Configurator : public OMPLPlannerConfigurator
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Fraction of time focused on boarder (0.0,1.] */
  double border_fraction = 0.9;

  /** @brief When extending motion fails, scale score by factor. */
  double failed_expansion_score_factor = 0.5;

  /** @brief Accept partially valid moves above fraction. */
  double min_valid_path_fraction = 0.5;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct KPIECE1Configurator : public OMPLPlannerConfigurator
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

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct RRTConfigurator : public OMPLPlannerConfigurator
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct RRTConnectConfigurator : public OMPLPlannerConfigurator
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct RRTstarConfigurator : public OMPLPlannerConfigurator
{
  /** @brief Max motion added to tree */
  double range = 0;

  /** @brief When close to goal select goal, with this probability. */
  double goal_bias = 0.05;

  /** @brief Stop collision checking as soon as C-free parent found. */
  bool delay_collision_checking = true;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct TRRTConfigurator : public OMPLPlannerConfigurator
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

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct PRMConfigurator : public OMPLPlannerConfigurator
{
  /** @brief Use k nearest neighbors. */
  int max_nearest_neighbors = 10;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct PRMstarConfigurator : public OMPLPlannerConfigurator
{
  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct LazyPRMstarConfigurator : public OMPLPlannerConfigurator
{
  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

struct SPARSConfigurator : public OMPLPlannerConfigurator
{
  /** @brief The maximum number of failures before terminating the algorithm */
  int max_failures = 1000;

  /** @brief Dense graph connection distance as a fraction of max. extent */
  double dense_delta_fraction = 0.001;

  /** @brief Sparse Roadmap connection distance as a fraction of max. extent */
  double sparse_delta_fraction = 0.25;

  /** @brief The stretch factor in terms of graph spanners for SPARS to check against */
  double stretch_factor = 3;

  /** @brief Create the planner */
  ompl::base::PlannerPtr create(ompl::base::SpaceInformationPtr si) const override;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_PLANNER_CONFIGURATOR_H
