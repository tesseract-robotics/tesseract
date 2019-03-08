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
#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/console.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_planning/trajopt/trajopt_array_planner.h>
#include <tesseract_planning/trajopt/trajopt_planner.h>

// This is probably an issue
#include <tesseract_ros/ros_basic_plotting.h>
#include <trajopt/plot_callback.hpp>

using namespace trajopt;

namespace tesseract
{
namespace tesseract_planning
{
bool TrajOptArrayPlanner::solve(PlannerResponse& response, const TrajOptArrayPlannerConfig& config)
{
  // Check that parameters are valid
  if (config.env_ == nullptr)
    throw std::invalid_argument("In trajopt_array_planner: env_ is a required parameter and has not been set");
  if (config.kin_ == nullptr)
    throw std::invalid_argument("In trajopt_array_planner: kin_ is a required parameter and has not been set");

  // -------- Construct the problem ------------
  // -------------------------------------------
  ProblemConstructionInfo pci(config.env_);
  pci.kin = config.kin_;

  // Populate Basic Info
  pci.basic_info.n_steps = static_cast<int>(config.target_waypoints_.size());
  pci.basic_info.manip = config.manipulator_;
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;

  // Populate Init Info
  pci.init_info.type = config.init_type_;
  if (config.init_type_ == trajopt::InitInfo::GIVEN_TRAJ)
    pci.init_info.data = config.seed_trajectory_;

  // Add constraints
  for (std::size_t ind = 0; ind < config.target_waypoints_.size(); ind++)
  {
    auto waypoint_type = config.target_waypoints_[ind]->getType();
    switch (waypoint_type)
    {
      case tesseract::tesseract_planning::JOINT_WAYPOINT:
      {
        JointWaypointPtr joint_waypoint = std::static_pointer_cast<JointWaypoint>(config.target_waypoints_[ind]);
        // Add initial joint position constraint
        std::shared_ptr<JointPosTermInfo> jv = std::shared_ptr<JointPosTermInfo>(new JointPosTermInfo);
        jv->coeffs = std::vector<double>(pci.kin->numJoints(), 1.0);
        jv->targets = joint_waypoint->joint_positions_;
        jv->first_step = static_cast<int>(ind);
        jv->last_step = static_cast<int>(ind);
        jv->name = "joint_position";
        jv->term_type = TT_CNT;
        pci.cnt_infos.push_back(jv);
        break;
      }
      case tesseract::tesseract_planning::CARTESIAN_WAYPOINT:
      {
        CartesianWaypointPtr cart_waypoint = std::static_pointer_cast<CartesianWaypoint>(config.target_waypoints_[ind]);
        std::shared_ptr<CartPoseTermInfo> pose = std::shared_ptr<CartPoseTermInfo>(new CartPoseTermInfo);
        pose->term_type = TT_CNT;
        pose->name = "cartesian_position";
        pose->link = config.end_effector_;
        pose->timestep = static_cast<int>(ind);
        pose->xyz = cart_waypoint->getPosition();
        pose->wxyz = cart_waypoint->getOrientation();
        pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
        pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
        pci.cnt_infos.push_back(pose);
        break;
      }
    }
  }

  // Set costs for the rest of the points
  if (config.collision_check_)
  {
    std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
    collision->name = "collision_cost";
    collision->term_type = TT_COST;
    collision->continuous = config.collision_continuous_;
    collision->first_step = 0;
    collision->last_step = pci.basic_info.n_steps - 1;
    collision->gap = 1;
    collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, config.collision_safety_margin_, 20);
    pci.cost_infos.push_back(collision);
  }
  if (config.smooth_velocities_)
  {
    std::shared_ptr<JointVelTermInfo> jv = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
    jv->coeffs = std::vector<double>(pci.kin->numJoints(), 5.0);
    jv->targets = std::vector<double>(pci.kin->numJoints(), 0.0);
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cost";
    jv->term_type = TT_COST;
    pci.cost_infos.push_back(jv);
  }
  if (config.smooth_accelerations_)
  {
    std::shared_ptr<JointAccTermInfo> ja = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
    ja->coeffs = std::vector<double>(pci.kin->numJoints(), 1.0);
    ja->targets = std::vector<double>(pci.kin->numJoints(), 0.0);
    ja->first_step = 0;
    ja->last_step = pci.basic_info.n_steps - 1;
    ja->name = "joint_accel_cost";
    ja->term_type = TT_COST;
    pci.cost_infos.push_back(ja);
  }
  if (config.smooth_jerks_)
  {
    std::shared_ptr<JointJerkTermInfo> jj = std::shared_ptr<JointJerkTermInfo>(new JointJerkTermInfo);
    jj->coeffs = std::vector<double>(pci.kin->numJoints(), 1.0);
    jj->targets = std::vector<double>(pci.kin->numJoints(), 0.0);
    jj->first_step = 0;
    jj->last_step = pci.basic_info.n_steps - 1;
    jj->name = "joint_jerk_cost";
    jj->term_type = TT_COST;
    pci.cost_infos.push_back(jj);
  }
  trajopt::TrajOptProbPtr prob = ConstructProblem(pci);

  // -------- Solve the problem ------------
  // ---------------------------------------
  // Set the parameters in trajopt_planner
  tesseract::tesseract_planning::TrajOptPlannerConfig config_planner(prob);
  config_planner.params = config.params_;
  config_planner.callbacks = config.callbacks_;

  // Create Plot Callback
  if (config.plot_callback_)
  {
    tesseract::tesseract_ros::ROSBasicPlottingPtr plotter_ptr(
        new tesseract::tesseract_ros::ROSBasicPlotting(config.env_));
    config_planner.callbacks.push_back(PlotCallback(*prob, plotter_ptr));
  }

  tesseract::tesseract_planning::TrajOptPlanner planner;
  tesseract::tesseract_planning::PlannerResponse planning_response;

  // Solve problem. Results are stored in the response
  bool success = planner.solve(planning_response, config_planner);
  response = planning_response;

  return success;
}

bool TrajOptArrayPlanner::terminate() { return false; }
void TrajOptArrayPlanner::clear() { request_ = PlannerRequest(); }
}  // namespace tesseract_planning
}  // namespace tesseract
