/**
 * @file ompl_freespace_planner.hpp
 * @brief Tesseract OMPL TrajOpt Freespace Planner Implementation
 *
 * @author Michael Ripperger
 * @date October 3, 2019
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
#include <tesseract_motion_planners/hybrid/ompl_trajopt_freespace_planner.h>

namespace tesseract_motion_planners
{
template <typename PlannerType>
OMPLTrajOptFreespacePlanner<PlannerType>::OMPLTrajOptFreespacePlanner(std::string name)
  : MotionPlanner(name), status_category_(std::make_shared<tesseract_common::GeneralStatusCategory>(name))
{
}

template <typename PlannerType>
bool OMPLTrajOptFreespacePlanner<PlannerType>::setConfiguration(
    OMPLFreespacePlannerConfig<PlannerType> ompl_config,
    std::shared_ptr<TrajOptPlannerFreespaceConfig> trajopt_config)
{
  // Set the number of states to be equal for the OMPL and trajopt planner configurations
  if (ompl_config.n_output_states > trajopt_config->num_steps)
  {
    trajopt_config->num_steps = ompl_config.n_output_states;
  }
  else
  {
    ompl_config.n_output_states = trajopt_config->num_steps;
  }

  bool success = true;
  if (!ompl_planner_.setConfiguration(ompl_config))
    success = false;

  if (!trajopt_planner_.setConfiguration(trajopt_config))
    success = false;
  else
    trajopt_config_ = trajopt_config;

  return success;
}

template <typename PlannerType>
bool OMPLTrajOptFreespacePlanner<PlannerType>::terminate()
{
  bool success = true;
  if (!ompl_planner_.terminate())
  {
    CONSOLE_BRIDGE_logError("Failed to terminate OMPL freespace planner");
    success = false;
  }

  if (!trajopt_planner_.terminate())
  {
    CONSOLE_BRIDGE_logError("Failed to terminate TrajOpt planner");
    success = false;
  }

  return success;
}

template <typename PlannerType>
void OMPLTrajOptFreespacePlanner<PlannerType>::clear()
{
  ompl_planner_.clear();
  trajopt_planner_.clear();
}

template <typename PlannerType>
tesseract_common::StatusCode OMPLTrajOptFreespacePlanner<PlannerType>::isConfigured() const
{
  tesseract_common::StatusCode ompl_status = ompl_planner_.isConfigured();
  tesseract_common::StatusCode trajopt_status = trajopt_planner_.isConfigured();

  if (!ompl_status && !trajopt_status)
    return tesseract_common::StatusCode(status_category_->IsNotConfigured, status_category_);

  if (!ompl_status)
    return ompl_status;

  if (!trajopt_status)
    return trajopt_status;

  return tesseract_common::StatusCode(status_category_->IsConfigured, status_category_);
}

template <typename PlannerType>
tesseract_common::StatusCode OMPLTrajOptFreespacePlanner<PlannerType>::solve(PlannerResponse& response, bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  // Solve problem using OMPL. Results are stored in the response
  tesseract_motion_planners::PlannerResponse ompl_planning_response;
  tesseract_common::StatusCode ompl_status = ompl_planner_.solve(ompl_planning_response, verbose);
  if (!ompl_status)
  {
    response = std::move(ompl_planning_response);
    response.status = ompl_status;
    return response.status;
  }

  if (trajopt_config_)
  {
    // The trajectory from OMPL may not be the same size as the number of steps requested in the TrajOpt config
    trajopt_config_->init_type = trajopt::InitInfo::GIVEN_TRAJ;
    trajopt_config_->seed_trajectory = ompl_planning_response.joint_trajectory.trajectory;
  }
  else
  {
    CONSOLE_BRIDGE_logError("Failed to set initial trajectory; ensure DescartesTrajOptArrayPlanner is configured with "
                            "TrajOptPlannerDefaultConfig");
    return tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::FailedToParseConfig, status_category_);
  }

  trajopt_planner_.setConfiguration(trajopt_config_);

  tesseract_motion_planners::PlannerResponse trajopt_planning_response;
  tesseract_common::StatusCode trajopt_status = trajopt_planner_.solve(trajopt_planning_response, verbose);
  response = std::move(trajopt_planning_response);
  response.status = trajopt_status;

  return response.status;
}

}  // namespace tesseract_motion_planners
