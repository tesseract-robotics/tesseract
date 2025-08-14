/**
 * @file utils.cpp
 * @brief Tesseract Environment Utility Functions.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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

namespace tesseract_environment
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
                                 const tesseract_scene_graph::SceneGraph& scene_graph,
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
      if (scene_graph.getInboundJoints(child_link)[0]->type != tesseract_scene_graph::JointType::FIXED)
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, true);
      else
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
}

void checkTrajectorySegment(tesseract_collision::ContactResultMap& contact_results,
                            tesseract_collision::ContinuousContactManager& manager,
                            const tesseract_common::TransformMap& state0,
                            const tesseract_common::TransformMap& state1,
                            const tesseract_collision::ContactRequest& contact_request)
{
  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state0.at(link_name), state1.at(link_name));

  manager.contactTest(contact_results, contact_request);
}

void checkTrajectoryState(tesseract_collision::ContactResultMap& contact_results,
                          tesseract_collision::DiscreteContactManager& manager,
                          const tesseract_common::TransformMap& state,
                          const tesseract_collision::ContactRequest& contact_request)
{
  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state.at(link_name));

  manager.contactTest(contact_results, contact_request);
}

void checkTrajectoryState(tesseract_collision::ContactResultMap& contact_results,
                          tesseract_collision::ContinuousContactManager& manager,
                          const tesseract_common::TransformMap& state,
                          const tesseract_collision::ContactRequest& contact_request)
{
  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state.at(link_name), state.at(link_name));

  manager.contactTest(contact_results, contact_request);
}

void printContinuousDebugInfo(const std::vector<std::string>& joint_names,
                              const Eigen::VectorXd& swp0,
                              const Eigen::VectorXd& swp1,
                              tesseract_common::TrajArray::Index step_idx,
                              tesseract_common::TrajArray::Index step_size,
                              tesseract_common::TrajArray::Index sub_step_idx = -1)
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
                            tesseract_common::TrajArray::Index step_idx,
                            tesseract_common::TrajArray::Index step_size,
                            tesseract_common::TrajArray::Index sub_step_idx = -1)
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

using CalcStateFn = std::function<tesseract_common::TransformMap(const Eigen::VectorXd& state)>;

tesseract_collision::TrajectoryContactResult
checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                tesseract_collision::ContinuousContactManager& manager,
                const CalcStateFn& state_fn,
                const std::vector<std::string>& joint_names,
                const tesseract_common::TrajArray& traj,
                const tesseract_collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract_collision::CollisionEvaluatorType::CONTINUOUS &&
      config.type != tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
    throw std::runtime_error("checkTrajectory was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type");

  if (traj.rows() < 2)
    throw std::runtime_error("checkTrajectory was given continuous contact manager with a trajectory that only has one "
                             "state.");

  bool debug_logging = console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;

  tesseract_collision::ContactTrajectoryResults::UPtr traj_contacts;
  if (debug_logging)
  {
    traj_contacts =
        std::make_unique<tesseract_collision::ContactTrajectoryResults>(joint_names, static_cast<int>(traj.rows()));
  }

  contacts.clear();
  contacts.reserve(static_cast<std::size_t>(traj.rows()));

  /** @brief Making this thread_local does not help because it is not called enough during planning */
  tesseract_collision::ContactResultMap state_results;
  tesseract_collision::ContactResultMap sub_state_results;

  // Estimate potential contacts for efficient memory allocation
  tesseract_collision::TrajectoryContactResult result(static_cast<std::size_t>(traj.rows()));

  if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::START_ONLY)
  {
    tesseract_common::TransformMap state = state_fn(traj.row(0));
    sub_state_results.clear();
    checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      result.addContact(0);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, false);
      if (debug_logging)
        printContinuousDebugInfo(joint_names, traj.row(0), traj.row(0), 0, traj.rows() - 1);
    }

    contacts.push_back(state_results);
    return result;
  }

  if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::END_ONLY)
  {
    tesseract_common::TransformMap state = state_fn(traj.row(traj.rows() - 1));
    sub_state_results.clear();
    tesseract_environment::checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      result.addContact(static_cast<int>(traj.rows() - 1));
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, false);
      if (debug_logging)
        printContinuousDebugInfo(joint_names, traj.row(traj.rows() - 1), traj.row(traj.rows() - 1), 0, traj.rows() - 1);
    }
    contacts.push_back(state_results);
    return result;
  }

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    for (tesseract_common::TrajArray::Index iStep = 0; iStep < traj.rows() - 1; ++iStep)
    {
      state_results.clear();

      double dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();
      if (dist > config.longest_valid_segment_length)
      {
        tesseract_common::TrajArray::Index cnt =
            static_cast<tesseract_common::TrajArray::Index>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, traj.cols());
        for (tesseract_common::TrajArray::Index iVar = 0; iVar < traj.cols(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

        tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;

        if (debug_logging)
        {
          step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(
              static_cast<int>(iStep + 1), traj.row(iStep), traj.row(iStep + 1), static_cast<int>(subtraj.rows()));
        }

        auto sub_segment_last_index = static_cast<int>(subtraj.rows() - 1);

        // Update start index based on collision check program mode
        tesseract_common::TrajArray::Index start_idx{ 0 };
        tesseract_common::TrajArray::Index end_idx{ subtraj.rows() - 1 };
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            ++start_idx;
        }

        if (iStep == (traj.rows() - 2))
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            --end_idx;
        }

        for (tesseract_common::TrajArray::Index iSubStep = start_idx; iSubStep < end_idx; ++iSubStep)
        {
          tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
          if (debug_logging)
          {
            substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(
                static_cast<int>(iSubStep) + 1, subtraj.row(iSubStep), subtraj.row(iSubStep + 1));
          }

          tesseract_common::TransformMap state0 = state_fn(subtraj.row(iSubStep));
          tesseract_common::TransformMap state1 = state_fn(subtraj.row(iSubStep + 1));
          sub_state_results.clear();
          checkTrajectorySegment(sub_state_results, manager, state0, state1, config.contact_request);
          if (!sub_state_results.empty())
          {
            result.addContact(static_cast<int>(iStep), static_cast<int>(iSubStep));

            if (debug_logging)
            {
              substep_contacts->contacts = sub_state_results;
              step_contacts->substeps[static_cast<size_t>(iSubStep)] = *substep_contacts;
            }
            double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
            // Always use addInterpolatedCollisionResults so cc_type is defined correctly
            state_results.addInterpolatedCollisionResults(sub_state_results,
                                                          iSubStep,
                                                          sub_segment_last_index,
                                                          manager.getActiveCollisionObjects(),
                                                          segment_dt,
                                                          false);
          }

          if (result && config.exit_on_first_contact)
            break;
        }
        contacts.push_back(state_results);

        if (result && config.exit_on_first_contact)
          break;
      }
      else
      {
        // Update start and end index based on collision check program mode
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.emplace_back();
            continue;
          }
        }
        if (iStep == (traj.rows() - 2))
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            continue;
        }

        tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
        tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;

        if (debug_logging)
        {
          step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(
              static_cast<int>(iStep + 1), traj.row(iStep), traj.row(iStep + 1), 1);
          substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(
              1, traj.row(iStep), traj.row(iStep + 1));
        }

        tesseract_common::TransformMap state0 = state_fn(traj.row(iStep));
        tesseract_common::TransformMap state1 = state_fn(traj.row(iStep + 1));
        checkTrajectorySegment(state_results, manager, state0, state1, config.contact_request);
        if (!state_results.empty())
        {
          result.addContact(static_cast<int>(iStep + 1));
          // For continuous no lvs addInterpolatedCollisionResults should not be used.

          if (debug_logging)
          {
            substep_contacts->contacts = state_results;
            step_contacts->substeps[0] = *substep_contacts;
            traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
          }
        }
        contacts.push_back(state_results);

        if (debug_logging)
        {
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }

        if (result && config.exit_on_first_contact)
          break;
      }
    }
  }
  else
  {
    tesseract_common::TrajArray::Index start_idx{ 0 };
    tesseract_common::TrajArray::Index end_idx{ traj.rows() - 1 };
    if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
        config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
    {
      contacts.emplace_back();
      ++start_idx;
    }

    if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
        config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
      --end_idx;

    for (tesseract_common::TrajArray::Index iStep = start_idx; iStep < end_idx; ++iStep)
    {
      tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
      tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
      if (debug_logging)
      {
        step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(
            static_cast<int>(iStep + 1), traj.row(iStep), traj.row(iStep + 1), 1);
        substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(
            1, traj.row(iStep), traj.row(iStep + 1));
      }

      tesseract_common::TransformMap state0 = state_fn(traj.row(iStep));
      tesseract_common::TransformMap state1 = state_fn(traj.row(iStep + 1));

      state_results.clear();
      checkTrajectorySegment(state_results, manager, state0, state1, config.contact_request);
      if (!state_results.empty())
      {
        result.addContact(static_cast<int>(iStep));
        // For continuous no lvs addInterpolatedCollisionResults should not be used.

        if (debug_logging)
        {
          substep_contacts->contacts = state_results;
          step_contacts->substeps[0] = *substep_contacts;
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }
      }
      contacts.push_back(state_results);

      if (debug_logging)
      {
        traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
      }

      if (result && config.exit_on_first_contact)
        break;
    }
  }

  if (result && debug_logging)
    std::cout << traj_contacts->trajectoryCollisionResultsTable().str();

  return result;
}

tesseract_collision::TrajectoryContactResult
checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                tesseract_collision::ContinuousContactManager& manager,
                const tesseract_scene_graph::StateSolver& state_solver,
                const std::vector<std::string>& joint_names,
                const tesseract_common::TrajArray& traj,
                const tesseract_collision::CollisionCheckConfig& config)
{
  CalcStateFn state_fn = [&joint_names, &state_solver](const Eigen::VectorXd& state) {
    return state_solver.getState(joint_names, state).link_transforms;
  };

  return checkTrajectory(contacts, manager, state_fn, joint_names, traj, config);
}

tesseract_collision::TrajectoryContactResult
checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                tesseract_collision::ContinuousContactManager& manager,
                const tesseract_kinematics::JointGroup& manip,
                const tesseract_common::TrajArray& traj,
                const tesseract_collision::CollisionCheckConfig& config)
{
  CalcStateFn state_fn = [&manip](const Eigen::VectorXd& state) { return manip.calcFwdKin(state); };

  const std::vector<std::string> joint_names = manip.getJointNames();
  return checkTrajectory(contacts, manager, state_fn, joint_names, traj, config);
}

tesseract_collision::TrajectoryContactResult
checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                tesseract_collision::DiscreteContactManager& manager,
                const CalcStateFn& state_fn,
                const std::vector<std::string>& joint_names,
                const tesseract_common::TrajArray& traj,
                const tesseract_collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract_collision::CollisionEvaluatorType::DISCRETE &&
      config.type != tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
    throw std::runtime_error("checkTrajectory was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type");

  if (traj.rows() == 0)
    throw std::runtime_error("checkTrajectory was given continuous contact manager with empty trajectory.");

  bool debug_logging = console_bridge::getLogLevel() < console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;

  tesseract_collision::ContactTrajectoryResults::UPtr traj_contacts;
  if (debug_logging)
  {
    traj_contacts =
        std::make_unique<tesseract_collision::ContactTrajectoryResults>(joint_names, static_cast<int>(traj.rows()));
  }

  contacts.clear();
  contacts.reserve(static_cast<std::size_t>(traj.rows()));

  /** @brief Making this thread_local does not help because it is not called enough during planning */
  tesseract_collision::ContactResultMap state_results;
  tesseract_collision::ContactResultMap sub_state_results;

  // Estimate potential contacts for efficient memory allocation
  tesseract_collision::TrajectoryContactResult result(static_cast<std::size_t>(traj.rows()));

  if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::START_ONLY)
  {
    tesseract_common::TransformMap state = state_fn(traj.row(0));
    sub_state_results.clear();
    tesseract_environment::checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      result.addContact(0);
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      if (debug_logging)
        printDiscreteDebugInfo(joint_names, traj.row(0), 0, traj.rows() - 1);
    }
    contacts.push_back(state_results);
    return result;
  }

  if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::END_ONLY)
  {
    tesseract_common::TransformMap state = state_fn(traj.row(traj.rows() - 1));
    sub_state_results.clear();
    tesseract_environment::checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      result.addContact(static_cast<int>(traj.rows() - 1));
      // Always use addInterpolatedCollisionResults so cc_type is defined correctly
      state_results.addInterpolatedCollisionResults(
          sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      if (debug_logging)
        printDiscreteDebugInfo(joint_names, traj.row(traj.rows() - 1), 0, traj.rows() - 1);
    }
    contacts.push_back(state_results);
    return result;
  }

  if (traj.rows() == 1)
  {
    tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
    tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
    if (debug_logging)
    {
      step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(1, traj.row(0));
      substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(1, traj.row(0));
    }

    if (config.check_program_mode != tesseract_collision::CollisionCheckProgramType::ALL)
      return result;

    auto sub_segment_last_index = static_cast<int>(traj.rows() - 1);
    state_results.clear();
    tesseract_common::TransformMap state = state_fn(traj.row(0));
    sub_state_results.clear();
    checkTrajectoryState(sub_state_results, manager, state, config.contact_request);

    if (!sub_state_results.empty())
    {
      result.addContact(0);
    }

    if (debug_logging)
    {
      substep_contacts->contacts = sub_state_results;
      step_contacts->substeps[0] = *substep_contacts;
      traj_contacts->steps[0] = *step_contacts;
    }

    double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
    state_results.addInterpolatedCollisionResults(
        sub_state_results, 0, sub_segment_last_index, manager.getActiveCollisionObjects(), segment_dt, true);
    contacts.push_back(state_results);

    if (result && debug_logging)
      std::cout << traj_contacts->trajectoryCollisionResultsTable().str();

    return result;
  }

  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    for (int iStep = 0; iStep < (traj.rows() - 1); ++iStep)
    {
      state_results.clear();

      const double dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();
      if (dist > config.longest_valid_segment_length)
      {
        int cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, traj.cols());
        for (tesseract_common::TrajArray::Index iVar = 0; iVar < traj.cols(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

        tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;

        if (debug_logging)
        {
          step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(
              iStep + 1, traj.row(iStep), traj.row(iStep + 1), static_cast<int>(subtraj.rows()));
        }

        auto sub_segment_last_index = static_cast<int>(subtraj.rows() - 1);

        // Update start index based on collision check program mode
        tesseract_common::TrajArray::Index start_idx{ 0 };
        tesseract_common::TrajArray::Index end_idx{ subtraj.rows() - 1 };
        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            ++start_idx;
        }

        if (iStep == (traj.rows() - 2))
        {
          // This is the last segment so check the last state
          end_idx = subtraj.rows();
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
            --end_idx;
        }

        for (tesseract_common::TrajArray::Index iSubStep = start_idx; iSubStep < end_idx; ++iSubStep)
        {
          tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
          if (debug_logging)
          {
            substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(
                static_cast<int>(iSubStep) + 1, subtraj.row(iSubStep));
          }

          tesseract_common::TransformMap state = state_fn(subtraj.row(iSubStep));
          sub_state_results.clear();
          checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
          if (!sub_state_results.empty())
          {
            result.addContact(iStep, static_cast<int>(iSubStep));
            if (debug_logging)
            {
              substep_contacts->contacts = sub_state_results;
              step_contacts->substeps[static_cast<size_t>(iSubStep)] = *substep_contacts;
            }
            double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
            state_results.addInterpolatedCollisionResults(sub_state_results,
                                                          iSubStep,
                                                          sub_segment_last_index,
                                                          manager.getActiveCollisionObjects(),
                                                          segment_dt,
                                                          true);
          }

          if (result && (config.exit_on_first_contact)
            break;
        }
        contacts.push_back(state_results);

          if (result && config.exit_on_first_contact)
          break;
      }
      else
      {
        // Special case when using LVS and only two states
        tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
        tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
        tesseract_collision::ContactTrajectorySubstepResults::UPtr end_substep_contacts;
        if (debug_logging)
        {
          step_contacts =
              std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(iStep + 1, traj.row(iStep));
          substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(1, traj.row(iStep));
          end_substep_contacts =
              std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(2, traj.row(iStep + 1));
        }

        if (iStep == 0 && traj.rows() == 2)
        {
          if (config.check_program_mode != tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START &&
              config.check_program_mode != tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            tesseract_common::TransformMap state = state_fn(traj.row(iStep));
            sub_state_results.clear();
            checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
            if (!sub_state_results.empty())
            {
              result.addContact(iStep);
              if (debug_logging)
              {
                substep_contacts->contacts = sub_state_results;
                step_contacts->substeps[0] = *substep_contacts;
                traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
              }
              state_results.addInterpolatedCollisionResults(
                  sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
            }

            if (result && config.exit_on_first_contact)
            {
              contacts.push_back(state_results);
              break;
            }
          }

          if (config.check_program_mode != tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END &&
              config.check_program_mode != tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            tesseract_common::TransformMap state = state_fn(traj.row(iStep + 1));
            sub_state_results.clear();
            checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
            if (!sub_state_results.empty())
            {
              result.addContact(static_cast<int>(iStep + 1));
              if (debug_logging)
              {
                end_substep_contacts->contacts = sub_state_results;
                step_contacts->substeps[1] = *end_substep_contacts;
                traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
              }
              state_results.addInterpolatedCollisionResults(
                  sub_state_results, 1, 1, manager.getActiveCollisionObjects(), 1, true);
            }

            if (result && config.exit_on_first_contact)
            {
              contacts.push_back(state_results);
              break;
            }
          }

          contacts.push_back(state_results);
          break;
        }

        if (iStep == 0)
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.emplace_back();
            continue;
          }
        }

        tesseract_common::TransformMap state = state_fn(traj.row(iStep));
        sub_state_results.clear();
        checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
        if (!sub_state_results.empty())
        {
          result.addContact(iStep);
          if (debug_logging)
          {
            substep_contacts->contacts = sub_state_results;
            step_contacts->substeps[0] = *substep_contacts;
            traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
          }
          state_results.addInterpolatedCollisionResults(
              sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
        }

        if (result && config.exit_on_first_contact)
        {
          contacts.push_back(state_results);
          break;
        }

        // If last segment check the end state
        if (iStep == (traj.rows() - 2))
        {
          if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
              config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
          {
            contacts.push_back(state_results);
            continue;
          }

          tesseract_common::TransformMap state = state_fn(traj.row(iStep + 1));
          sub_state_results.clear();
          checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
          if (!sub_state_results.empty())
          {
            result.addContact(static_cast<int>(iStep + 1));
            if (debug_logging)
            {
              end_substep_contacts->contacts = sub_state_results;
              step_contacts->substeps[1] = *end_substep_contacts;
              traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
            }
            state_results.addInterpolatedCollisionResults(
                sub_state_results, 1, 1, manager.getActiveCollisionObjects(), 1, true);
          }

          if (result && config.exit_on_first_contact)
          {
            contacts.push_back(state_results);
            break;
          }
        }

        contacts.push_back(state_results);

        if (debug_logging)
        {
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }
      }
    }
  }
  else
  {
    tesseract_common::TrajArray::Index start_idx{ 0 };
    tesseract_common::TrajArray::Index end_idx(traj.rows());

    if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_START ||
        config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
    {
      contacts.emplace_back();
      ++start_idx;
    }

    if (config.check_program_mode == tesseract_collision::CollisionCheckProgramType::ALL_EXCEPT_END ||
        config.check_program_mode == tesseract_collision::CollisionCheckProgramType::INTERMEDIATE_ONLY)
      --end_idx;

    for (tesseract_common::TrajArray::Index iStep = start_idx; iStep < end_idx; ++iStep)
    {
      tesseract_collision::ContactTrajectoryStepResults::UPtr step_contacts;
      tesseract_collision::ContactTrajectorySubstepResults::UPtr substep_contacts;
      if (debug_logging)
      {
        step_contacts = std::make_unique<tesseract_collision::ContactTrajectoryStepResults>(static_cast<int>(iStep + 1),
                                                                                            traj.row(iStep));
        substep_contacts = std::make_unique<tesseract_collision::ContactTrajectorySubstepResults>(1, traj.row(iStep));
      }

      state_results.clear();

      tesseract_common::TransformMap state = state_fn(traj.row(iStep));
      sub_state_results.clear();
      checkTrajectoryState(sub_state_results, manager, state, config.contact_request);
      if (!sub_state_results.empty())
      {
        result.addContact(static_cast<int>(iStep));
        if (debug_logging)
        {
          substep_contacts->contacts = sub_state_results;
          step_contacts->substeps[0] = *substep_contacts;
          traj_contacts->steps[static_cast<size_t>(iStep)] = *step_contacts;
        }
        state_results.addInterpolatedCollisionResults(
            sub_state_results, 0, 0, manager.getActiveCollisionObjects(), 0, true);
      }
      contacts.push_back(state_results);

      if (result && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  if (result && debug_logging)
    std::cout << traj_contacts->trajectoryCollisionResultsTable().str();

  return result;
}

tesseract_collision::TrajectoryContactResult
checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                tesseract_collision::DiscreteContactManager& manager,
                const tesseract_scene_graph::StateSolver& state_solver,
                const std::vector<std::string>& joint_names,
                const tesseract_common::TrajArray& traj,
                const tesseract_collision::CollisionCheckConfig& config)
{
  CalcStateFn state_fn = [&joint_names, &state_solver](const Eigen::VectorXd& state) {
    return state_solver.getState(joint_names, state).link_transforms;
  };

  return checkTrajectory(contacts, manager, state_fn, joint_names, traj, config);
}

tesseract_collision::TrajectoryContactResult
checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                tesseract_collision::DiscreteContactManager& manager,
                const tesseract_kinematics::JointGroup& manip,
                const tesseract_common::TrajArray& traj,
                const tesseract_collision::CollisionCheckConfig& config)
{
  CalcStateFn state_fn = [&manip](const Eigen::VectorXd& state) { return manip.calcFwdKin(state); };

  const std::vector<std::string> joint_names = manip.getJointNames();
  return checkTrajectory(contacts, manager, state_fn, joint_names, traj, config);
}

}  // namespace tesseract_environment
