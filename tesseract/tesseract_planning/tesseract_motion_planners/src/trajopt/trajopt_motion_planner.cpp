/**
 * @file trajopt_planner.cpp
 * @brief Tesseract ROS Trajopt planner
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <tesseract_environment/core/utils.h>
#include <boost/date_time/posix_time/posix_time.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>

using namespace trajopt;

static const double LONGEST_VALID_SEGMENT_FRACTION_DEFAULT = 0.01;

namespace tesseract_motion_planners
{
TrajOptMotionPlannerStatusCategory::TrajOptMotionPlannerStatusCategory(std::string name) : name_(std::move(name)) {}
const std::string& TrajOptMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string TrajOptMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case IsConfigured:
    {
      return "Is Configured";
    }
    case SolutionFound:
    {
      return "Found valid solution";
    }
    case IsNotConfigured:
    {
      return "Planner is not configured, must call setConfiguration prior to calling solve.";
    }
    case FailedToParseConfig:
    {
      return "Failed to parse config data";
    }
    case FailedToFindValidSolution:
    {
      return "Failed to find valid solution";
    }
    case FoundValidSolutionInCollision:
    {
      return "Found valid solution, but is in collision";
    }
    default:
    {
      assert(false);
      return "";
    }
  }
}

TrajOptMotionPlanner::TrajOptMotionPlanner(std::string name)
  : MotionPlanner(std::move(name))
  , config_(nullptr)
  , status_category_(std::make_shared<const TrajOptMotionPlannerStatusCategory>(name_))
{
}

bool TrajOptMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing optimization is not implemented yet");
  return false;
}

void TrajOptMotionPlanner::clear()
{
  request_ = PlannerRequest();
  config_ = nullptr;
}

tesseract_common::StatusCode TrajOptMotionPlanner::solve(PlannerResponse& response,
                                                         PostPlanCheckType check_type,
                                                         bool verbose)
{
  tesseract_common::StatusCode config_status = isConfigured();
  if (!config_status)
  {
    response.status = config_status;
    CONSOLE_BRIDGE_logError("Planner %s is not configured", name_.c_str());
    return config_status;
  }

  // Set Log Level
  if (verbose)
    util::gLogLevel = util::LevelInfo;
  else
    util::gLogLevel = util::LevelWarn;

  // Create optimizer
  sco::BasicTrustRegionSQP opt(config_->prob);
  opt.setParameters(config_->params);
  opt.initialize(trajToDblVec(config_->prob->GetInitTraj()));

  // Add all callbacks
  for (const sco::Optimizer::Callback& callback : config_->callbacks)
  {
    opt.addCallback(callback);
  }

  // Optimize
  auto tStart = boost::posix_time::second_clock::local_time();
  opt.optimize();
  CONSOLE_BRIDGE_logInform("planning time: %.3f", (boost::posix_time::second_clock::local_time() - tStart).seconds());

  // Check and report collisions
  const Eigen::MatrixX2d& limits = config_->prob->GetKin()->getLimits();
  double length = 0;
  double extent = (limits.col(1) - limits.col(0)).norm();
  if (config_->longest_valid_segment_fraction > 0 && config_->longest_valid_segment_length > 0)
  {
    length = std::min(config_->longest_valid_segment_fraction * extent, config_->longest_valid_segment_length);
  }
  else if (config_->longest_valid_segment_fraction > 0)
  {
    length = config_->longest_valid_segment_fraction * extent;
  }
  else if (config_->longest_valid_segment_length > 0)
  {
    length = config_->longest_valid_segment_length;
  }
  else
  {
    length = LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
  }

  std::vector<tesseract_collision::ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = config_->prob->GetEnv()->getStateSolver();
  tesseract_collision::ContinuousContactManager::Ptr continuous_manager =
      config_->prob->GetEnv()->getContinuousContactManager();
  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      config_->prob->GetEnv()->getSceneGraph(),
      config_->prob->GetKin()->getActiveLinkNames(),
      config_->prob->GetEnv()->getCurrentState()->link_transforms);

  validator_ = std::make_shared<TrajectoryValidator>(config_->prob->GetEnv()->getContinuousContactManager(),
                                                     config_->prob->GetEnv()->getDiscreteContactManager(),
                                                     length,
                                                     verbose);

  bool valid = validator_->trajectoryValid(
      getTraj(opt.x(), config_->prob->GetVars()), check_type, *state_solver, config_->prob->GetKin()->getJointNames());

  // Send response
  response.joint_trajectory.trajectory = getTraj(opt.x(), config_->prob->GetVars());
  response.joint_trajectory.joint_names = config_->prob->GetKin()->getJointNames();
  if (opt.results().status != sco::OptStatus::OPT_CONVERGED)
  {
    response.status =
        tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::FailedToFindValidSolution, status_category_);
  }
  else if (!valid)
  {
    response.status = tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::FoundValidSolutionInCollision,
                                                   status_category_);
  }
  else
  {
    response.status = tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::SolutionFound, status_category_);
    CONSOLE_BRIDGE_logInform("Final trajectory is collision free");
  }

  return response.status;
}

tesseract_common::StatusCode TrajOptMotionPlanner::isConfigured() const
{
  if (config_ != nullptr && config_->prob != nullptr)
    return tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::IsConfigured, status_category_);

  return tesseract_common::StatusCode(TrajOptMotionPlannerStatusCategory::IsNotConfigured, status_category_);
}

bool TrajOptMotionPlanner::setConfiguration(TrajOptPlannerConfig::Ptr config)
{
  config_ = std::move(config);
  return config_->generate();
}

}  // namespace tesseract_motion_planners
