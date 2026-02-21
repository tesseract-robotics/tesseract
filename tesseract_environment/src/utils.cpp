/**
 * @file utils.cpp
 * @brief Tesseract Environment Utility Functions.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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

#include <tesseract_collision/core/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

namespace tesseract::environment
{
/**
 * @brief Get the active Link Names Recursively
 *
 *        This currently only works for graphs that are trees. Need to create a generic method using boost::visitor
 *        TODO: Need to update using graph->getLinkChildren
 *
 * @param active_links
 * @param scene_graph
 * @param current_link
 * @param active
 */
void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                 const tesseract::scene_graph::SceneGraph& scene_graph,
                                 const std::string& current_link,
                                 bool active)
{
  // recursively get all active links
  assert(scene_graph.isTree());
  if (active)
  {
    active_links.push_back(current_link);
    for (const auto& child_link : scene_graph.getAdjacentLinkNames(current_link))
      getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
  else
  {
    for (const auto& child_link : scene_graph.getAdjacentLinkNames(current_link))
      if (scene_graph.getInboundJoints(child_link)[0]->type != tesseract::scene_graph::JointType::FIXED)
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, true);
      else
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
}

void checkTrajectorySegment(tesseract::collision::ContactResultMap& contact_results,
                            tesseract::collision::ContinuousContactManager& manager,
                            const tesseract::common::TransformMap& state0,
                            const tesseract::common::TransformMap& state1,
                            const tesseract::collision::ContactRequest& contact_request)
{
  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state0.at(link_name), state1.at(link_name));

  manager.contactTest(contact_results, contact_request);
}

void checkTrajectoryState(tesseract::collision::ContactResultMap& contact_results,
                          tesseract::collision::DiscreteContactManager& manager,
                          const tesseract::common::TransformMap& state,
                          const tesseract::collision::ContactRequest& contact_request)
{
  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state.at(link_name));

  manager.contactTest(contact_results, contact_request);
}

void checkTrajectoryState(tesseract::collision::ContactResultMap& contact_results,
                          tesseract::collision::ContinuousContactManager& manager,
                          const tesseract::common::TransformMap& state,
                          const tesseract::collision::ContactRequest& contact_request)
{
  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state.at(link_name), state.at(link_name));

  manager.contactTest(contact_results, contact_request);
}

void printContinuousDebugInfo(const std::vector<std::string>& joint_names,
                              const Eigen::VectorXd& swp0,
                              const Eigen::VectorXd& swp1,
                              tesseract::common::TrajArray::Index step_idx,
                              tesseract::common::TrajArray::Index step_size,
                              tesseract::common::TrajArray::Index sub_step_idx = -1)
{
  std::stringstream ss;
  ss << "Continuous collision detected at step: " << step_idx << " of " << step_size;
  if (sub_step_idx >= 0)
    ss << " substep: " << sub_step_idx;
  ss << "\n";

  ss << "     Names:";
  for (const auto& name : joint_names)
    ss << " " << name;

  ss << "\n"
     << "    State0: " << swp0 << "\n"
     << "    State1: " << swp1 << "\n";

  CONSOLE_BRIDGE_logDebug(ss.str().c_str());
}

void printDiscreteDebugInfo(const std::vector<std::string>& joint_names,
                            const Eigen::VectorXd& swp,
                            tesseract::common::TrajArray::Index step_idx,
                            tesseract::common::TrajArray::Index step_size,
                            tesseract::common::TrajArray::Index sub_step_idx = -1)
{
  std::stringstream ss;
  ss << "Discrete collision detected at step: " << step_idx << " of " << step_size;
  if (sub_step_idx >= 0)
    ss << " substep: " << sub_step_idx;
  ss << "\n";

  ss << "     Names:";
  for (const auto& name : joint_names)
    ss << " " << name;

  ss << "\n"
     << "    State: " << swp << "\n";

  CONSOLE_BRIDGE_logDebug(ss.str().c_str());
}

using CalcStateFn = std::function<tesseract::common::TransformMap(const Eigen::VectorXd& state)>;

tesseract::collision::ContactTrajectoryResults
checkTrajectory(std::vector<tesseract::collision::ContactResultMap>& contacts,
                tesseract::collision::ContinuousContactManager& manager,
                const CalcStateFn& state_fn,
                const std::vector<std::string>& joint_names,
                const tesseract::common::TrajArray& traj,
                const tesseract::collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract::collision::CollisionEvaluatorType::CONTINUOUS &&
      config.type != tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    throw std::runtime_error("checkTrajectory was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type");

  if (traj.rows() < 2)
    throw std::runtime_error("checkTrajectory was given continuous contact manager with a trajectory that only has one "
                             "state.");

  bool debug_logging = console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;

  tesseract::collision::ContactTrajectoryResults traj_contacts(joint_names, static_cast<int>(traj.rows()));

  contacts.clear();
  contacts.reserve(static_cast<std::size_t>(traj.rows()));

  /** @brief Making this thread_local does not help because it is not called enough during planning */
  tesseract::collision::ContactResultMap state_results;
  tesseract::collision::ContactResultMap sub_state_results;

  if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::START_ONLY)
  {
    tesseract::common::TransformMap state = state_fn(traj.row(0));
    sub_state_results.clear();
    checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(0, 0, 1, traj.row(0), traj.row(0), traj.row(0), traj.row(0), sub_state_results);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, false);
      if (debug_logging)
        printContinuousDebugInfo(joint_names, traj.row(0), traj.row(0), 0, traj.rows() - 1);
    }

    contacts.push_back(state_results);
    return traj_contacts;
  }

  if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::END_ONLY)
  {
    tesseract::common::TransformMap state = state_fn(traj.row(traj.rows() - 1));
    sub_state_results.clear();
    tesseract::environment::checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(static_cast<int>(traj.rows() - 1),
                               0,
                               1,
                               traj.row(traj.rows() - 1),
                               traj.row(traj.rows() - 1),
                               traj.row(traj.rows() - 1),
                               traj.row(traj.rows() - 1),
                               sub_state_results);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, false);
      if (debug_logging)
        printContinuousDebugInfo(joint_names, traj.row(traj.rows() - 1), traj.row(traj.rows() - 1), 0, traj.rows() - 1);
    }
    contacts.push_back(state_results);
    return traj_contacts;
  }

  if (config.type == tesseract::collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    for (tesseract::common::TrajArray::Index iStep = 0; iStep < traj.rows() - 1; ++iStep)
    {
      state_results.clear();

      double dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();
      if (dist > config.longest_valid_segment_length)
      {
        tesseract::common::TrajArray::Index cnt =
            static_cast<tesseract::common::TrajArray::Index>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract::common::TrajArray subtraj(cnt, traj.cols());
        for (tesseract::common::TrajArray::Index iVar = 0; iVar < traj.cols(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

        tesseract::collision::ContactTrajectoryStepResults::UPtr step_contacts =
            std::make_unique<tesseract::collision::ContactTrajectoryStepResults>(
                static_cast<int>(iStep + 1), traj.row(iStep), traj.row(iStep + 1), static_cast<int>(subtraj.rows()));

        auto sub_segment_last_index = static_cast<int>(subtraj.rows() - 1);

        // Update start index based on collision check program mode
        tesseract::common::TrajArray::Index start_idx{ 0 };
        tesseract::common::TrajArray::Index end_idx{ subtraj.rows() - 1 };
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            ++start_idx;
        }

        if (iStep == (traj.rows() - 2))
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            --end_idx;
        }

        for (tesseract::common::TrajArray::Index iSubStep = start_idx; iSubStep < end_idx; ++iSubStep)
        {
          tesseract::common::TransformMap state0 = state_fn(subtraj.row(iSubStep));
          tesseract::common::TransformMap state1 = state_fn(subtraj.row(iSubStep + 1));
          sub_state_results.clear();
          checkTrajectorySegment(sub_state_results, manager, state0, state1, config.contact_request);
          if (!sub_state_results.empty())
          {
            traj_contacts.addContact(static_cast<int>(iStep),
                                     static_cast<int>(iSubStep),
                                     static_cast<int>(end_idx),
                                     traj.row(iStep),
                                     traj.row(iStep + 1),
                                     subtraj.row(iSubStep),
                                     subtraj.row(iSubStep + 1),
                                     sub_state_results);

            double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
            // Always use addInterpolatedCollisionResults so cc_type is defined correctly
            state_results.addInterpolatedCollisionResults(sub_state_results,
                                                          iSubStep,
                                                          sub_segment_last_index,
                                                          manager.getActiveCollisionObjects(),
                                                          segment_dt,
                                                          false);

            // Exit behavior
            if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST)
            {
              // Break out of substep loop and outer step loop
              iSubStep = end_idx;  // ensure exit
            }
            else if (config.exit_condition == tesseract::collision::CollisionCheckExitType::ONE_PER_STEP)
            {
              // Stop checking further substates for this step
              break;
            }
          }
        }
        contacts.push_back(state_results);

        // After finishing substeps, possibly exit outer loop
        if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST && !state_results.empty())
          break;
      }
      else
      {
        // Update start and end index based on collision check program mode
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.emplace_back();
            continue;
          }
        }
        if (iStep == (traj.rows() - 2))
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            continue;
        }

        tesseract::common::TransformMap state0 = state_fn(traj.row(iStep));
        tesseract::common::TransformMap state1 = state_fn(traj.row(iStep + 1));
        checkTrajectorySegment(state_results, manager, state0, state1, config.contact_request);
        contacts.push_back(state_results);
        if (!state_results.empty())
        {
          traj_contacts.addContact(static_cast<int>(iStep),
                                   0,
                                   1,
                                   traj.row(iStep),
                                   traj.row(iStep + 1),
                                   traj.row(iStep),
                                   traj.row(iStep + 1),
                                   state_results);
          // For continuous no lvs addInterpolatedCollisionResults should not be used.

          if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST)
            break;
        }
      }
    }
  }
  else
  {
    tesseract::common::TrajArray::Index start_idx{ 0 };
    tesseract::common::TrajArray::Index end_idx{ traj.rows() - 1 };
    if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
        config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
    {
      contacts.emplace_back();
      ++start_idx;
    }

    if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
        config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
      --end_idx;

    for (tesseract::common::TrajArray::Index iStep = start_idx; iStep < end_idx; ++iStep)
    {
      tesseract::common::TransformMap state0 = state_fn(traj.row(iStep));
      tesseract::common::TransformMap state1 = state_fn(traj.row(iStep + 1));

      state_results.clear();
      checkTrajectorySegment(state_results, manager, state0, state1, config.contact_request);
      contacts.push_back(state_results);
      if (!state_results.empty())
      {
        traj_contacts.addContact(static_cast<int>(iStep),
                                 0,
                                 1,
                                 traj.row(iStep),
                                 traj.row(iStep + 1),
                                 traj.row(iStep),
                                 traj.row(iStep + 1),
                                 state_results);
        // For continuous no lvs addInterpolatedCollisionResults should not be used.

        if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST && !state_results.empty())
          break;
      }
    }
  }

  if (traj_contacts && debug_logging)
    std::cout << traj_contacts.trajectoryCollisionResultsTable().str();

  return traj_contacts;
}

tesseract::collision::ContactTrajectoryResults
checkTrajectory(std::vector<tesseract::collision::ContactResultMap>& contacts,
                tesseract::collision::ContinuousContactManager& manager,
                const tesseract::scene_graph::StateSolver& state_solver,
                const std::vector<std::string>& joint_names,
                const tesseract::common::TrajArray& traj,
                const tesseract::collision::CollisionCheckConfig& config)
{
  CalcStateFn state_fn = [&joint_names, &state_solver](const Eigen::VectorXd& state) {
    return state_solver.getState(joint_names, state).link_transforms;
  };

  return checkTrajectory(contacts, manager, state_fn, joint_names, traj, config);
}

tesseract::collision::ContactTrajectoryResults
checkTrajectory(std::vector<tesseract::collision::ContactResultMap>& contacts,
                tesseract::collision::ContinuousContactManager& manager,
                const tesseract::kinematics::JointGroup& manip,
                const tesseract::common::TrajArray& traj,
                const tesseract::collision::CollisionCheckConfig& config)
{
  CalcStateFn state_fn = [&manip](const Eigen::VectorXd& state) { return manip.calcFwdKin(state); };

  const std::vector<std::string> joint_names = manip.getJointNames();
  return checkTrajectory(contacts, manager, state_fn, joint_names, traj, config);
}

tesseract::collision::ContactTrajectoryResults
checkTrajectory(std::vector<tesseract::collision::ContactResultMap>& contacts,
                tesseract::collision::DiscreteContactManager& manager,
                const CalcStateFn& state_fn,
                const std::vector<std::string>& joint_names,
                const tesseract::common::TrajArray& traj,
                const tesseract::collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract::collision::CollisionEvaluatorType::DISCRETE &&
      config.type != tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE)
    throw std::runtime_error("checkTrajectory was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type");

  if (traj.rows() == 0)
    throw std::runtime_error("checkTrajectory was given continuous contact manager with empty trajectory.");

  bool debug_logging = console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;

  tesseract::collision::ContactTrajectoryResults traj_contacts(joint_names, static_cast<int>(traj.rows()));

  contacts.clear();
  contacts.reserve(static_cast<std::size_t>(traj.rows()));

  /** @brief Making this thread_local does not help because it is not called enough during planning */
  tesseract::collision::ContactResultMap state_results;
  tesseract::collision::ContactResultMap sub_state_results;

  if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::START_ONLY)
  {
    tesseract::common::TransformMap state = state_fn(traj.row(0));
    sub_state_results.clear();
    tesseract::environment::checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(0, 0, 1, traj.row(0), traj.row(0), traj.row(0), traj.row(0), sub_state_results);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      if (debug_logging)
        printDiscreteDebugInfo(joint_names, traj.row(0), 0, traj.rows() - 1);
    }
    contacts.push_back(state_results);
    return traj_contacts;
  }

  if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::END_ONLY)
  {
    tesseract::common::TransformMap state = state_fn(traj.row(traj.rows() - 1));
    sub_state_results.clear();
    tesseract::environment::checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(static_cast<int>(traj.rows() - 1),
                               0,
                               1,
                               traj.row(traj.rows() - 1),
                               traj.row(traj.rows() - 1),
                               traj.row(traj.rows() - 1),
                               traj.row(traj.rows() - 1),
                               sub_state_results);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      if (debug_logging)
        printDiscreteDebugInfo(joint_names, traj.row(traj.rows() - 1), 0, traj.rows() - 1);
    }
    contacts.push_back(state_results);
    return traj_contacts;
  }

  if (traj.rows() == 1)
  {
    if (config.check_program_mode != tesseract::collision::CollisionCheckProgramType::ALL)
      return traj_contacts;

    auto sub_segment_last_index = static_cast<int>(traj.rows() - 1);
    state_results.clear();
    tesseract::common::TransformMap state = state_fn(traj.row(0));
    sub_state_results.clear();
    checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      traj_contacts.addContact(0, 0, 1, traj.row(0), traj.row(0), traj.row(0), traj.row(0), sub_state_results);
    }

    double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
    state_results.addInterpolatedCollisionResults(
        sub_state_results, 0, sub_segment_last_index, manager.getActiveCollisionObjects(), segment_dt, true);
    contacts.push_back(state_results);

    if (traj_contacts && debug_logging)
      std::cout << traj_contacts.trajectoryCollisionResultsTable().str();

    return traj_contacts;
  }

  if (config.type == tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    for (int iStep = 0; iStep < (traj.rows() - 1); ++iStep)
    {
      state_results.clear();

      const double dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();
      if (dist > config.longest_valid_segment_length)
      {
        int cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract::common::TrajArray subtraj(cnt, traj.cols());
        for (tesseract::common::TrajArray::Index iVar = 0; iVar < traj.cols(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

        auto sub_segment_last_index = static_cast<int>(subtraj.rows() - 1);

        // Update start index based on collision check program mode
        tesseract::common::TrajArray::Index start_idx{ 0 };
        tesseract::common::TrajArray::Index end_idx{ subtraj.rows() - 1 };
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            ++start_idx;
        }

        for (tesseract::common::TrajArray::Index iSubStep = start_idx; iSubStep < end_idx; ++iSubStep)
        {
          tesseract::common::TransformMap state = state_fn(subtraj.row(iSubStep));
          sub_state_results.clear();
          checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
          if (!sub_state_results.empty())
          {
            traj_contacts.addContact(static_cast<int>(iStep),
                                     static_cast<int>(iSubStep),
                                     static_cast<int>(end_idx),
                                     traj.row(iStep),
                                     traj.row(iStep + 1),
                                     subtraj.row(iSubStep),
                                     subtraj.row(iSubStep),
                                     sub_state_results);
            double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
            state_results.addInterpolatedCollisionResults(sub_state_results,
                                                          iSubStep,
                                                          sub_segment_last_index,
                                                          manager.getActiveCollisionObjects(),
                                                          segment_dt,
                                                          true);
            if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST)
            {
              // Break out of substep loop and outer step loop
              iSubStep = end_idx;  // ensure exit
            }
            else if (config.exit_condition == tesseract::collision::CollisionCheckExitType::ONE_PER_STEP)
            {
              // Stop checking further substates for this step
              break;
            }
          }
        }
        contacts.push_back(state_results);

        if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST && !state_results.empty())
          break;
      }
      else
      {
        // Special case for two state trajectory
        if (iStep == 0 && traj.rows() == 2)
        {
          if (config.check_program_mode != tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START &&
              config.check_program_mode != tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            tesseract::common::TransformMap state = state_fn(traj.row(iStep));
            sub_state_results.clear();
            checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
            if (!sub_state_results.empty())
            {
              traj_contacts.addContact(static_cast<int>(iStep),
                                       0,
                                       1,
                                       traj.row(iStep),
                                       traj.row(iStep),
                                       traj.row(iStep),
                                       traj.row(iStep),
                                       sub_state_results);
              state_results.addInterpolatedCollisionResults(
                  sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
              if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST)
              {
                contacts.push_back(state_results);
                break;
              }
            }
            contacts.push_back(state_results);
          }

          if (config.check_program_mode != tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END &&
              config.check_program_mode != tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            // Need to add empty start state if checking the end state and start was skipped
            if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START)
              contacts.push_back(state_results);

            state_results.clear();
            tesseract::common::TransformMap state = state_fn(traj.row(iStep + 1));
            sub_state_results.clear();
            checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
            if (!sub_state_results.empty())
            {
              traj_contacts.addContact(static_cast<int>(iStep + 1),
                                       0,
                                       1,
                                       traj.row(iStep + 1),
                                       traj.row(iStep + 1),
                                       traj.row(iStep + 1),
                                       traj.row(iStep + 1),
                                       sub_state_results);
              state_results.addInterpolatedCollisionResults(
                  sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
              if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST)
              {
                contacts.push_back(state_results);
                break;
              }
            }
            contacts.push_back(state_results);
          }
          break;
        }

        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.emplace_back();
            continue;
          }
        }

        tesseract::common::TransformMap state = state_fn(traj.row(iStep));
        sub_state_results.clear();
        checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
        if (!sub_state_results.empty())
        {
          traj_contacts.addContact(static_cast<int>(iStep),
                                   0,
                                   1,
                                   traj.row(iStep),
                                   traj.row(iStep),
                                   traj.row(iStep),
                                   traj.row(iStep),
                                   sub_state_results);
          state_results.addInterpolatedCollisionResults(
              sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);

          // Exit behavior
          if (config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST)
          {
            contacts.push_back(state_results);
            break;
          }
        }

        contacts.push_back(state_results);
      }
      // If last segment check the end state
      if (iStep == (traj.rows() - 2))
      {
        state_results.clear();
        if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
            config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
        {
          continue;
        }

        tesseract::common::TransformMap state = state_fn(traj.row(iStep + 1));
        sub_state_results.clear();
        checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
        if (!sub_state_results.empty())
        {
          traj_contacts.addContact(static_cast<int>(iStep + 1),
                                   0,
                                   1,
                                   traj.row(iStep + 1),
                                   traj.row(iStep + 1),
                                   traj.row(iStep + 1),
                                   traj.row(iStep + 1),
                                   sub_state_results);
          state_results.addInterpolatedCollisionResults(
              sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
        }
        contacts.push_back(state_results);
      }
    }
  }
  else
  {
    tesseract::common::TrajArray::Index start_idx{ 0 };
    tesseract::common::TrajArray::Index end_idx(traj.rows());

    if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
        config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
    {
      contacts.emplace_back();
      ++start_idx;
    }

    if (config.check_program_mode == tesseract::collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
        config.check_program_mode == tesseract::collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
      --end_idx;

    for (tesseract::common::TrajArray::Index iStep = start_idx; iStep < end_idx; ++iStep)
    {
      state_results.clear();

      tesseract::common::TransformMap state = state_fn(traj.row(iStep));
      sub_state_results.clear();
      checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
      if (!sub_state_results.empty())
      {
        traj_contacts.addContact(static_cast<int>(iStep),
                                 0,
                                 1,
                                 traj.row(iStep),
                                 traj.row(iStep),
                                 traj.row(iStep),
                                 traj.row(iStep),
                                 sub_state_results);
        state_results.addInterpolatedCollisionResults(
            sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      }
      contacts.push_back(state_results);

      if (!sub_state_results.empty() && config.exit_condition == tesseract::collision::CollisionCheckExitType::FIRST)
        break;
    }
  }

  if (traj_contacts && debug_logging)
    std::cout << traj_contacts.trajectoryCollisionResultsTable().str();

  return traj_contacts;
}

tesseract::collision::ContactTrajectoryResults
checkTrajectory(std::vector<tesseract::collision::ContactResultMap>& contacts,
                tesseract::collision::DiscreteContactManager& manager,
                const tesseract::scene_graph::StateSolver& state_solver,
                const std::vector<std::string>& joint_names,
                const tesseract::common::TrajArray& traj,
                const tesseract::collision::CollisionCheckConfig& config)
{
  CalcStateFn state_fn = [&joint_names, &state_solver](const Eigen::VectorXd& state) {
    return state_solver.getState(joint_names, state).link_transforms;
  };

  return checkTrajectory(contacts, manager, state_fn, joint_names, traj, config);
}

tesseract::collision::ContactTrajectoryResults
checkTrajectory(std::vector<tesseract::collision::ContactResultMap>& contacts,
                tesseract::collision::DiscreteContactManager& manager,
                const tesseract::kinematics::JointGroup& manip,
                const tesseract::common::TrajArray& traj,
                const tesseract::collision::CollisionCheckConfig& config)
{
  CalcStateFn state_fn = [&manip](const Eigen::VectorXd& state) { return manip.calcFwdKin(state); };

  const std::vector<std::string> joint_names = manip.getJointNames();
  return checkTrajectory(contacts, manager, state_fn, joint_names, traj, config);
}

}  // namespace tesseract::environment
