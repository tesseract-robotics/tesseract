/**
 * @file descartes_trajopt_array_planner.cpp
 * @brief Tesseract ROS Descartes TrajOpt hybrid planner
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
#include <tesseract_motion_planners/hybrid/descartes_trajopt_array_planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
DescartesTrajOptArrayPlanner<FloatType>::DescartesTrajOptArrayPlanner(std::string name)
  : MotionPlanner(std::move(name)), status_category_(std::make_shared<tesseract_common::GeneralStatusCategory>(name_))
{
}

template <typename FloatType>
bool DescartesTrajOptArrayPlanner<FloatType>::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

template <typename FloatType>
void DescartesTrajOptArrayPlanner<FloatType>::clear()
{
  request_ = PlannerRequest();
  trajopt_planner_.clear();
  descartes_planner_.clear();
  trajopt_config_ = nullptr;
}

template <typename FloatType>
tesseract_common::StatusCode DescartesTrajOptArrayPlanner<FloatType>::isConfigured() const
{
  tesseract_common::StatusCode descartes_status = descartes_planner_.isConfigured();
  tesseract_common::StatusCode trajopt_status = trajopt_planner_.isConfigured();

  if (!descartes_status && !trajopt_status)
    return tesseract_common::StatusCode(status_category_->IsNotConfigured, status_category_);

  if (!descartes_status)
    return descartes_status;

  if (!trajopt_status)
    return trajopt_status;

  return tesseract_common::StatusCode(status_category_->IsConfigured, status_category_);
}

template <typename FloatType>
tesseract_common::StatusCode DescartesTrajOptArrayPlanner<FloatType>::solve(PlannerResponse& response, bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  // Solve problem using descartes. Results are stored in the response
  tesseract_motion_planners::PlannerResponse descartes_planning_response;
  tesseract_common::StatusCode descartes_status = descartes_planner_.solve(descartes_planning_response, verbose);
  if (!descartes_status &&
      (descartes_status.value() != DescartesMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision))
  {
    response = std::move(descartes_planning_response);
    return descartes_status;
  }

  std::shared_ptr<TrajOptPlannerDefaultConfig> config =
      std::dynamic_pointer_cast<TrajOptPlannerDefaultConfig>(trajopt_config_);
  if (config)
  {
    config->seed_trajectory = descartes_planning_response.joint_trajectory.trajectory;
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

  return trajopt_status;
}

template <typename FloatType>
bool DescartesTrajOptArrayPlanner<FloatType>::setConfiguration(
    const DescartesMotionPlannerConfig<FloatType>& descartes_config,
    const TrajOptPlannerConfig::Ptr& trajopt_config)
{
  bool success = true;
  if (!descartes_planner_.setConfiguration(descartes_config))
    success = false;

  if (!trajopt_planner_.setConfiguration(trajopt_config))
    success = false;
  else
    trajopt_config_ = trajopt_config;

  return success;
}
}  // namespace tesseract_motion_planners
