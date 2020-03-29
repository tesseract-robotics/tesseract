/**
 * @file trajopt_planner_freespace_config.cpp
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
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_freespace_config.h>
#include <tesseract_motion_planners/trajopt/config/utils.h>

namespace tesseract_motion_planners
{
std::shared_ptr<trajopt::ProblemConstructionInfo> TrajOptPlannerFreespaceConfig::generatePCI() const
{
  if (!checkUserInput())
    return nullptr;

  // -------- Construct the problem ------------
  // -------------------------------------------
  trajopt::ProblemConstructionInfo pci(tesseract);

  // Populate Basic Info
  if (!addBasicInfo(pci))
    return nullptr;

  pci.basic_info.n_steps = num_steps;

  if (!addInitTrajectory(pci))
    return nullptr;

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = tesseract->getEnvironmentConst();
  tesseract_environment::AdjacencyMap map(
      env->getSceneGraph(), pci.kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  const std::vector<std::string>& adjacency_links = map.getActiveLinkNames();

  // Add the first waypoint
  {
    WaypointTermInfo term_info;
    term_info = createWaypointTermInfo(
        target_waypoints.front(), 0, pci.kin->getJointNames(), adjacency_links, link, tcp.front());
    pci.cnt_infos.insert(pci.cnt_infos.end(), term_info.cnt.begin(), term_info.cnt.end());
    pci.cost_infos.insert(pci.cost_infos.end(), term_info.cost.begin(), term_info.cost.end());
  }
  // Add the last waypoint
  {
    WaypointTermInfo term_info;
    int ind = num_steps - 1;
    if (tcp.size() == target_waypoints.size())
    {
      term_info = createWaypointTermInfo(target_waypoints.back(),
                                         ind,
                                         pci.kin->getJointNames(),
                                         adjacency_links,
                                         link,
                                         tcp[static_cast<std::size_t>(ind)]);
    }
    else
    {
      term_info = createWaypointTermInfo(
          target_waypoints.back(), ind, pci.kin->getJointNames(), adjacency_links, link, tcp.front());
    }

    pci.cnt_infos.insert(pci.cnt_infos.end(), term_info.cnt.begin(), term_info.cnt.end());
    pci.cost_infos.insert(pci.cost_infos.end(), term_info.cost.begin(), term_info.cost.end());
  }

  /* Update the first and last step for the costs
   * Certain costs (collision checking and configuration) should not be applied to start and end states
   * that are incapable of changing (i.e. joint positions). Therefore, the first and last indices of these
   * costs (which equal 0 and num_steps-1 by default) should be changed to exclude those states
   */
  std::vector<int> fixed_steps;
  if (target_waypoints.front()->getType() == WaypointType::JOINT_WAYPOINT ||
      target_waypoints.front()->getType() == WaypointType::JOINT_TOLERANCED_WAYPOINT)
  {
    fixed_steps.push_back(0);
  }
  if (target_waypoints.back()->getType() == WaypointType::JOINT_WAYPOINT ||
      target_waypoints.back()->getType() == WaypointType::JOINT_TOLERANCED_WAYPOINT)
  {
    fixed_steps.push_back(num_steps - 1);
  }

  if (collision_constraint_config.enabled)
    addCollisionConstraint(pci, fixed_steps);

  if (collision_cost_config.enabled)
    addCollisionCost(pci, fixed_steps);

  if (smooth_velocities)
    addVelocitySmoothing(pci, fixed_steps);

  if (smooth_accelerations)
    addAccelerationSmoothing(pci, fixed_steps);

  if (smooth_jerks)
    addJerkSmoothing(pci, fixed_steps);

  if (configuration != nullptr)
    addKinematicConfiguration(pci, fixed_steps);

  if (!constraint_error_functions.empty())
    addConstraintErrorFunctions(pci, fixed_steps);

  return std::make_shared<trajopt::ProblemConstructionInfo>(pci);
}

}  // namespace tesseract_motion_planners
