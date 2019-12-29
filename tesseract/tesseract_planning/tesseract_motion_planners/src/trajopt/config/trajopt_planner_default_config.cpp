/**
 * @file trajopt_planner_default_config.cpp
 * @brief A TrajOpt planner configuration class with default values suitable for most applications
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
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
#include <tesseract_motion_planners/trajopt/config/utils.h>

static const double LONGEST_VALID_SEGMENT_FRACTION_DEFAULT = 0.01;

namespace tesseract_motion_planners
{
TrajOptPlannerDefaultConfig::TrajOptPlannerDefaultConfig(tesseract::Tesseract::ConstPtr tesseract_,
                                                         std::string manipulator_,
                                                         std::string link_,
                                                         tesseract_common::VectorIsometry3d tcp_)
  : tesseract(std::move(tesseract_)), manipulator(std::move(manipulator_)), link(std::move(link_)), tcp(std::move(tcp_))
{
}

TrajOptPlannerDefaultConfig::TrajOptPlannerDefaultConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                                                         const std::string& manipulator_,
                                                         const std::string& link_,
                                                         const Eigen::Isometry3d& tcp_)
  : TrajOptPlannerDefaultConfig(tesseract_, manipulator_, link_, tesseract_common::VectorIsometry3d(1, tcp_))
{
}

std::shared_ptr<trajopt::ProblemConstructionInfo> TrajOptPlannerDefaultConfig::generatePCI() const
{
  using namespace trajopt;

  // Check that parameters are valid
  if (tesseract == nullptr)
  {
    CONSOLE_BRIDGE_logError("In trajopt_array_planner: tesseract_ is a required parameter and has not been set");
    return nullptr;
  }

  if (target_waypoints.size() < 2)
  {
    CONSOLE_BRIDGE_logError("TrajOpt Planner Config requires at least 2 waypoints");
    return nullptr;
  }

  if (tcp.size() != target_waypoints.size() && tcp.size() != 1)
  {
    std::stringstream ss;
    ss << "Number of TCP transforms (" << tcp.size() << ") does not match the number of waypoints ("
       << target_waypoints.size() << ") and is also not 1";
    CONSOLE_BRIDGE_logError(ss.str().c_str());
    return nullptr;
  }

  // -------- Construct the problem ------------
  // -------------------------------------------
  ProblemConstructionInfo pci(tesseract);
  pci.kin = pci.getManipulator(manipulator);

  if (pci.kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("In trajopt_array_planner: manipulator_ does not exist in kin_map_");
    return nullptr;
  }

  // Populate Basic Info
  pci.basic_info.n_steps = static_cast<int>(target_waypoints.size());
  pci.basic_info.manip = manipulator;
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;
  pci.basic_info.convex_solver = optimizer;

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = tesseract->getEnvironmentConst();
  tesseract_kinematics::ForwardKinematics::ConstPtr kin =
      tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  tesseract_environment::AdjacencyMap map(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->transforms);
  const std::vector<std::string>& adjacency_links = map.getActiveLinkNames();

  // Populate Init Info
  pci.init_info.type = init_type;
  if (init_type == trajopt::InitInfo::GIVEN_TRAJ)
  {
    pci.init_info.data = seed_trajectory;

    // Add check to make sure if starts with joint waypoint that the seed trajectory also starts at this waypoint
    // If it does not start this causes issues in trajopt and it will never converge.
    if (isJointWaypointType(target_waypoints.front()->getType()))
    {
      const auto jwp = std::static_pointer_cast<JointWaypoint>(target_waypoints.front());
      const Eigen::VectorXd position = jwp->getPositions(kin->getJointNames());
      for (int i = 0; i < static_cast<int>(kin->numJoints()); ++i)
      {
        if (std::abs(position[i] - seed_trajectory(0, i)) > static_cast<double>(std::numeric_limits<float>::epsilon()))
        {
          std::stringstream ss;
          ss << "Seed trajectory start position does not match starting joint waypoint position!";
          ss << "    waypoint: " << position.transpose().matrix() << std::endl;
          ss << "  seed start: " << seed_trajectory.row(0).transpose().matrix() << std::endl;
          CONSOLE_BRIDGE_logError(ss.str().c_str());
          return nullptr;
        }
      }
    }
  }

  // Add constraints
  for (std::size_t ind = 0; ind < target_waypoints.size(); ind++)
  {
    WaypointTermInfo term_info;
    if (tcp.size() == target_waypoints.size())
    {
      term_info = createWaypointTermInfo(
          target_waypoints[ind], static_cast<int>(ind), pci.kin->getJointNames(), adjacency_links, link, tcp[ind]);
    }
    else
    {
      term_info = createWaypointTermInfo(
          target_waypoints[ind], static_cast<int>(ind), pci.kin->getJointNames(), adjacency_links, link, tcp.front());
    }

    pci.cnt_infos.insert(pci.cnt_infos.end(), term_info.cnt.begin(), term_info.cnt.end());
    pci.cost_infos.insert(pci.cost_infos.end(), term_info.cost.begin(), term_info.cost.end());
  }

  /* Update the first and last step for the costs
   * Certain costs (collision checking and configuration) should not be applied to start and end states
   * that are incapable of changing (i.e. joint positions). Therefore, the first and last indices of these
   * costs (which equal 0 and num_steps-1 by default) should be changed to exclude those states
   */
  int cost_first_step = 0;
  int cost_last_step = pci.basic_info.n_steps - 1;
  if (target_waypoints.front()->getType() == WaypointType::JOINT_WAYPOINT ||
      target_waypoints.front()->getType() == WaypointType::JOINT_TOLERANCED_WAYPOINT)
  {
    ++cost_first_step;
  }
  if (target_waypoints.back()->getType() == WaypointType::JOINT_WAYPOINT ||
      target_waypoints.back()->getType() == WaypointType::JOINT_TOLERANCED_WAYPOINT)
  {
    --cost_last_step;
  }

  // Set costs for the rest of the points
  if (collision_check)
  {
    // Calculate longest valid segment length
    const Eigen::MatrixX2d& limits = kin->getLimits();
    double length = 0;
    double extent = (limits.col(1) - limits.col(0)).norm();
    if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
    {
      length = std::min(longest_valid_segment_fraction * extent, longest_valid_segment_length);
    }
    else if (longest_valid_segment_fraction > 0)
    {
      length = longest_valid_segment_fraction * extent;
    }
    else if (longest_valid_segment_length > 0)
    {
      length = longest_valid_segment_length;
    }
    else
    {
      length = LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
    }

    // Create a default collision term info
    trajopt::TermInfo::Ptr ti = createCollisionTermInfo(pci.basic_info.n_steps,
                                                        collision_safety_margin,
                                                        collision_continuous,
                                                        collision_coeff,
                                                        contact_test_type,
                                                        length);

    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
    std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
    ct->first_step = cost_first_step;
    ct->last_step = cost_last_step;

    pci.cost_infos.push_back(ct);
  }
  if (smooth_velocities)
  {
    if (velocity_coeff.size() == 0)
      pci.cost_infos.push_back(
          createSmoothVelocityTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints())));
    else
      pci.cost_infos.push_back(createSmoothVelocityTermInfo(pci.basic_info.n_steps, velocity_coeff));
  }
  if (smooth_accelerations)
  {
    if (acceleration_coeff.size() == 0)
      pci.cost_infos.push_back(
          createSmoothAccelerationTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints())));
    else
      pci.cost_infos.push_back(createSmoothAccelerationTermInfo(pci.basic_info.n_steps, acceleration_coeff));
  }
  if (smooth_jerks)
  {
    if (jerk_coeff.size() == 0)
      pci.cost_infos.push_back(
          createSmoothJerkTermInfo(pci.basic_info.n_steps, static_cast<int>(pci.kin->numJoints())));
    else
      pci.cost_infos.push_back(createSmoothJerkTermInfo(pci.basic_info.n_steps, jerk_coeff));
  }
  if (configuration != nullptr)
  {
    trajopt::TermInfo::Ptr ti =
        createConfigurationTermInfo(configuration, pci.kin->getJointNames(), pci.basic_info.n_steps);

    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
    std::shared_ptr<trajopt::JointPosTermInfo> jp = std::static_pointer_cast<trajopt::JointPosTermInfo>(ti);
    jp->first_step = cost_first_step;
    jp->last_step = cost_last_step;

    pci.cost_infos.push_back(jp);
  }

  if (!constraint_error_functions.empty())
  {
    for (std::size_t i = 0; i < constraint_error_functions.size(); ++i)
    {
      auto& c = constraint_error_functions[i];
      trajopt::TermInfo::Ptr ti = createUserDefinedTermInfo(
          pci.basic_info.n_steps, std::get<0>(c), std::get<1>(c), "user_defined_" + std::to_string(i));

      // Update the term info with the (possibly) new start and end state indices for which to apply this cost
      std::shared_ptr<trajopt::UserDefinedTermInfo> ef = std::static_pointer_cast<trajopt::UserDefinedTermInfo>(ti);
      ef->term_type = trajopt::TT_CNT;
      ef->constraint_type = std::get<2>(c);
      ef->coeff = std::get<3>(c);
      ef->first_step = cost_first_step;
      ef->last_step = cost_last_step;

      pci.cnt_infos.push_back(ef);
    }
  }

  return std::make_shared<trajopt::ProblemConstructionInfo>(pci);
}

bool TrajOptPlannerDefaultConfig::generate()
{
  std::shared_ptr<trajopt::ProblemConstructionInfo> pci = generatePCI();
  if (!pci)
  {
    CONSOLE_BRIDGE_logError("Failed to construct problem from problem construction information");
    return false;
  }
  prob = trajopt::ConstructProblem(*pci);

  return true;
}

}  // namespace tesseract_motion_planners
