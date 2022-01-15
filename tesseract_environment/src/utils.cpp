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
#include <tesseract_environment/utils.h>

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

tesseract_collision::ContactResultMap checkTrajectorySegment(tesseract_collision::ContinuousContactManager& manager,
                                                             const tesseract_common::TransformMap& state0,
                                                             const tesseract_common::TransformMap& state1,
                                                             const tesseract_collision::CollisionCheckConfig& config)
{
  manager.applyContactManagerConfig(config.contact_manager_config);
  return checkTrajectorySegment(manager, state0, state1, config.contact_request);
}

tesseract_collision::ContactResultMap checkTrajectorySegment(tesseract_collision::ContinuousContactManager& manager,
                                                             const tesseract_common::TransformMap& state0,
                                                             const tesseract_common::TransformMap& state1,
                                                             const tesseract_collision::ContactRequest& contact_request)
{
  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state0.at(link_name), state1.at(link_name));

  tesseract_collision::ContactResultMap collisions;
  manager.contactTest(collisions, contact_request);

  if (!collisions.empty())
  {
    if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
    {
      for (auto& collision : collisions)
      {
        std::stringstream ss;
        ss << "Continuous collision detected between '" << collision.first.first << "' and '" << collision.first.second
           << "' with distance " << collision.second.front().distance << std::endl;

        CONSOLE_BRIDGE_logError(ss.str().c_str());
      }
    }
  }

  return collisions;
}

tesseract_collision::ContactResultMap checkTrajectoryState(tesseract_collision::DiscreteContactManager& manager,
                                                           const tesseract_common::TransformMap& state,
                                                           const tesseract_collision::CollisionCheckConfig& config)
{
  manager.applyContactManagerConfig(config.contact_manager_config);
  return checkTrajectoryState(manager, state, config.contact_request);
}

tesseract_collision::ContactResultMap checkTrajectoryState(tesseract_collision::DiscreteContactManager& manager,
                                                           const tesseract_common::TransformMap& state,
                                                           const tesseract_collision::ContactRequest& contact_request)
{
  tesseract_collision::ContactResultMap collisions;

  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state.at(link_name));

  manager.contactTest(collisions, contact_request);

  if (!collisions.empty())
  {
    if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
    {
      for (auto& collision : collisions)
      {
        std::stringstream ss;
        ss << "Discrete collision detected between '" << collision.first.first << "' and '" << collision.first.second
           << "' with distance " << collision.second.front().distance << std::endl;

        CONSOLE_BRIDGE_logError(ss.str().c_str());
      }
    }
  }

  return collisions;
}

/**
 * @brief This process contact results
 * @param segment_results
 * @param sub_segment_results
 * @param segment_index
 * @param segment_last_index
 * @param manip_active_link_names
 * @param discrete
 */
void processInterpolatedSubSegmentCollisionResults(tesseract_collision::ContactResultMap& segment_results,
                                                   tesseract_collision::ContactResultMap& sub_segment_results,
                                                   int sub_segment_index,
                                                   int sub_segment_last_index,
                                                   const std::vector<std::string>& manip_active_link_names,
                                                   bool discrete)
{
  double segment_dt = (sub_segment_last_index > 0) ? 1.0 / static_cast<double>(sub_segment_last_index) : 0.0;
  for (auto& pair : sub_segment_results)
  {
    // Update cc_time and cc_type
    for (auto& r : pair.second)
    {
      // Iterate over the two time values in r.cc_time
      for (size_t j = 0; j < 2; ++j)
      {
        if (std::find(manip_active_link_names.begin(), manip_active_link_names.end(), r.link_names[j]) !=
            manip_active_link_names.end())
        {
          r.cc_time[j] = (r.cc_time[j] < 0) ?
                             (static_cast<double>(sub_segment_index) * segment_dt) :
                             (static_cast<double>(sub_segment_index) * segment_dt) + (r.cc_time[j] * segment_dt);
          assert(r.cc_time[j] >= 0.0 && r.cc_time[j] <= 1.0);
          if (sub_segment_index == 0 &&
              (r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time0 || discrete))
            r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time0;
          else if (sub_segment_index == sub_segment_last_index &&
                   (r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time1 || discrete))
            r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time1;
          else
            r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Between;

          // If discrete set cc_transform for discrete continuous
          if (discrete)
            r.cc_transform = r.transform;
        }
      }

      if (sub_segment_last_index > 0)
      {
        // Add results to the full segment results
        auto it = segment_results.find(pair.first);
        if (it == segment_results.end())
          segment_results.insert(pair);
        else
          it->second.insert(it->second.end(), pair.second.begin(), pair.second.end());
      }
    }
  }

  if (sub_segment_last_index == 0)
    segment_results = sub_segment_results;
}

bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::ContinuousContactManager& manager,
                     const tesseract_scene_graph::StateSolver& state_solver,
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

  manager.applyContactManagerConfig(config.contact_manager_config);

  bool found = false;
  contacts.resize(static_cast<size_t>(traj.rows() - 1));
  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      double dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();
      if (dist > config.longest_valid_segment_length)
      {
        long cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, traj.cols());
        for (long iVar = 0; iVar < traj.cols(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_scene_graph::SceneState state0 = state_solver.getState(joint_names, subtraj.row(iSubStep));
          tesseract_scene_graph::SceneState state1 = state_solver.getState(joint_names, subtraj.row(iSubStep + 1));
          tesseract_collision::ContactResultMap sub_segment_results =
              checkTrajectorySegment(manager, state0.link_transforms, state1.link_transforms, config.contact_request);
          if (!sub_segment_results.empty())
          {
            found = true;
            processInterpolatedSubSegmentCollisionResults(segment_results,
                                                          sub_segment_results,
                                                          iSubStep,
                                                          static_cast<int>(subtraj.rows() - 1),
                                                          manager.getActiveCollisionObjects(),
                                                          false);

            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Continuous collision detected at step: " << iStep << " of " << (traj.rows() - 1)
                 << " substep: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : joint_names)
                ss << " " << name;

              ss << std::endl
                 << "    State0: " << subtraj.row(iSubStep) << std::endl
                 << "    State1: " << subtraj.row(iSubStep + 1) << std::endl;

              CONSOLE_BRIDGE_logError(ss.str().c_str());
            }
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        tesseract_scene_graph::SceneState state0 = state_solver.getState(joint_names, traj.row(iStep));
        tesseract_scene_graph::SceneState state1 = state_solver.getState(joint_names, traj.row(iStep + 1));
        segment_results =
            checkTrajectorySegment(manager, state0.link_transforms, state1.link_transforms, config.contact_request);
        if (!segment_results.empty())
        {
          found = true;
          if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
          {
            std::stringstream ss;
            ss << "Continuous collision detected at step: " << iStep << " of " << (traj.rows() - 1) << std::endl;

            ss << "     Names:";
            for (const auto& name : joint_names)
              ss << " " << name;

            ss << std::endl
               << "    State0: " << traj.row(iStep) << std::endl
               << "    State1: " << traj.row(iStep + 1) << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
    }
  }
  else
  {
    for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      tesseract_scene_graph::SceneState state0 = state_solver.getState(joint_names, traj.row(iStep));
      tesseract_scene_graph::SceneState state1 = state_solver.getState(joint_names, traj.row(iStep + 1));

      segment_results =
          checkTrajectorySegment(manager, state0.link_transforms, state1.link_transforms, config.contact_request);
      if (!segment_results.empty())
      {
        found = true;
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (traj.rows() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : joint_names)
            ss << " " << name;

          ss << std::endl
             << "    State0: " << traj.row(iStep) << std::endl
             << "    State1: " << traj.row(iStep + 1) << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  return found;
}

bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::ContinuousContactManager& manager,
                     const tesseract_kinematics::JointGroup& manip,
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

  manager.applyContactManagerConfig(config.contact_manager_config);

  bool found = false;
  contacts.resize(static_cast<size_t>(traj.rows() - 1));
  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_CONTINUOUS)
  {
    for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      double dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();
      if (dist > config.longest_valid_segment_length)
      {
        long cnt = static_cast<long>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, traj.cols());
        for (long iVar = 0; iVar < traj.cols(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_common::TransformMap state0 = manip.calcFwdKin(subtraj.row(iSubStep));
          tesseract_common::TransformMap state1 = manip.calcFwdKin(subtraj.row(iSubStep + 1));
          tesseract_collision::ContactResultMap sub_segment_results =
              checkTrajectorySegment(manager, state0, state1, config.contact_request);
          if (!sub_segment_results.empty())
          {
            found = true;
            processInterpolatedSubSegmentCollisionResults(segment_results,
                                                          sub_segment_results,
                                                          iSubStep,
                                                          static_cast<int>(subtraj.rows() - 1),
                                                          manager.getActiveCollisionObjects(),
                                                          false);

            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Continuous collision detected at step: " << iStep << " of " << (traj.rows() - 1)
                 << " substep: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : manip.getJointNames())
                ss << " " << name;

              ss << std::endl
                 << "    State0: " << subtraj.row(iSubStep) << std::endl
                 << "    State1: " << subtraj.row(iSubStep + 1) << std::endl;

              CONSOLE_BRIDGE_logError(ss.str().c_str());
            }
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        tesseract_common::TransformMap state0 = manip.calcFwdKin(traj.row(iStep));
        tesseract_common::TransformMap state1 = manip.calcFwdKin(traj.row(iStep + 1));
        segment_results = checkTrajectorySegment(manager, state0, state1, config.contact_request);
        if (!segment_results.empty())
        {
          found = true;
          if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
          {
            std::stringstream ss;
            ss << "Continuous collision detected at step: " << iStep << " of " << (traj.rows() - 1) << std::endl;

            ss << "     Names:";
            for (const auto& name : manip.getJointNames())
              ss << " " << name;

            ss << std::endl
               << "    State0: " << traj.row(iStep) << std::endl
               << "    State1: " << traj.row(iStep + 1) << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
    }
  }
  else
  {
    for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      tesseract_common::TransformMap state0 = manip.calcFwdKin(traj.row(iStep));
      tesseract_common::TransformMap state1 = manip.calcFwdKin(traj.row(iStep + 1));

      segment_results = checkTrajectorySegment(manager, state0, state1, config.contact_request);
      if (!segment_results.empty())
      {
        found = true;
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (traj.rows() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : manip.getJointNames())
            ss << " " << name;

          ss << std::endl
             << "    State0: " << traj.row(iStep) << std::endl
             << "    State1: " << traj.row(iStep + 1) << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  return found;
}

bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::DiscreteContactManager& manager,
                     const tesseract_scene_graph::StateSolver& state_solver,
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

  manager.applyContactManagerConfig(config.contact_manager_config);

  contacts.resize(static_cast<size_t>(traj.rows()));
  if (traj.rows() == 1)
  {
    tesseract_collision::ContactResultMap& state_results = contacts[0];
    state_results.clear();
    tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, traj.row(0));
    tesseract_collision::ContactResultMap sub_state_results =
        checkTrajectoryState(manager, state.link_transforms, config.contact_request);
    processInterpolatedSubSegmentCollisionResults(state_results,
                                                  sub_state_results,
                                                  0,
                                                  static_cast<int>(traj.rows() - 1),
                                                  manager.getActiveCollisionObjects(),
                                                  true);
    return (!state_results.empty());
  }

  bool found = false;
  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    for (int iStep = 0; iStep < traj.rows(); ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      double dist = -1;
      if (iStep < traj.rows() - 1)
        dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();

      if (dist > 0 && dist > config.longest_valid_segment_length)
      {
        int cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, traj.cols());
        for (long iVar = 0; iVar < traj.cols(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, subtraj.row(iSubStep));
          tesseract_collision::ContactResultMap sub_state_results =
              checkTrajectoryState(manager, state.link_transforms, config.contact_request);
          if (!sub_state_results.empty())
          {
            found = true;
            processInterpolatedSubSegmentCollisionResults(segment_results,
                                                          sub_state_results,
                                                          iSubStep,
                                                          static_cast<int>(subtraj.rows() - 1),
                                                          manager.getActiveCollisionObjects(),
                                                          true);

            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Discrete collision detected at step: " << iStep << " of " << (traj.rows() - 1)
                 << " substate: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : joint_names)
                ss << " " << name;

              ss << std::endl << "    State: " << subtraj.row(iSubStep) << std::endl;

              CONSOLE_BRIDGE_logError(ss.str().c_str());
            }
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, traj.row(iStep));
        tesseract_collision::ContactResultMap sub_segment_results =
            checkTrajectoryState(manager, state.link_transforms, config.contact_request);
        if (!sub_segment_results.empty())
        {
          found = true;
          processInterpolatedSubSegmentCollisionResults(
              segment_results, sub_segment_results, 0, 0, manager.getActiveCollisionObjects(), true);
          if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
          {
            std::stringstream ss;
            ss << "Discrete collision detected at step: " << iStep << " of " << (traj.rows() - 1) << std::endl;

            ss << "     Names:";
            for (const auto& name : joint_names)
              ss << " " << name;

            ss << std::endl << "    State: " << traj.row(iStep) << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
    }
  }
  else
  {
    for (int iStep = 0; iStep < traj.rows(); ++iStep)
    {
      tesseract_collision::ContactResultMap& state_results = contacts[static_cast<size_t>(iStep)];
      state_results.clear();

      tesseract_scene_graph::SceneState state = state_solver.getState(joint_names, traj.row(iStep));
      tesseract_collision::ContactResultMap sub_state_results =
          checkTrajectoryState(manager, state.link_transforms, config.contact_request);
      if (!sub_state_results.empty())
      {
        found = true;
        processInterpolatedSubSegmentCollisionResults(
            state_results, sub_state_results, 0, 0, manager.getActiveCollisionObjects(), true);
        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (traj.rows() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : joint_names)
            ss << " " << name;

          ss << std::endl << "    State0: " << traj.row(iStep) << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }
  return found;
}

bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::DiscreteContactManager& manager,
                     const tesseract_kinematics::JointGroup& manip,
                     const tesseract_common::TrajArray& traj,
                     const tesseract_collision::CollisionCheckConfig& config)
{
  if (config.type != tesseract_collision::CollisionEvaluatorType::DISCRETE &&
      config.type != tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
    throw std::runtime_error("checkTrajectory was given an CollisionEvaluatorType that is inconsistent with the "
                             "ContactManager type");

  if (traj.rows() == 0)
    throw std::runtime_error("checkTrajectory was given continuous contact manager with empty trajectory.");

  manager.applyContactManagerConfig(config.contact_manager_config);

  contacts.resize(static_cast<size_t>(traj.rows()));
  if (traj.rows() == 1)
  {
    tesseract_collision::ContactResultMap& state_results = contacts[0];
    state_results.clear();

    tesseract_common::TransformMap state = manip.calcFwdKin(traj.row(0));
    tesseract_collision::ContactResultMap sub_state_results =
        checkTrajectoryState(manager, state, config.contact_request);
    processInterpolatedSubSegmentCollisionResults(
        state_results, sub_state_results, 0, 0, manager.getActiveCollisionObjects(), true);
    return (!state_results.empty());
  }

  bool found = false;
  if (config.type == tesseract_collision::CollisionEvaluatorType::LVS_DISCRETE)
  {
    for (int iStep = 0; iStep < traj.rows(); ++iStep)
    {
      tesseract_collision::ContactResultMap& segment_results = contacts[static_cast<size_t>(iStep)];
      segment_results.clear();

      double dist = -1;
      if (iStep < traj.rows() - 1)
        dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();

      if (dist > 0 && dist > config.longest_valid_segment_length)
      {
        int cnt = static_cast<int>(std::ceil(dist / config.longest_valid_segment_length)) + 1;
        tesseract_common::TrajArray subtraj(cnt, traj.cols());
        for (long iVar = 0; iVar < traj.cols(); ++iVar)
          subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

        for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
        {
          tesseract_common::TransformMap state = manip.calcFwdKin(subtraj.row(iSubStep));
          tesseract_collision::ContactResultMap sub_state_results =
              checkTrajectoryState(manager, state, config.contact_request);
          if (!sub_state_results.empty())
          {
            found = true;
            processInterpolatedSubSegmentCollisionResults(segment_results,
                                                          sub_state_results,
                                                          iSubStep,
                                                          static_cast<int>(subtraj.rows() - 1),
                                                          manager.getActiveCollisionObjects(),
                                                          true);

            if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
            {
              std::stringstream ss;
              ss << "Discrete collision detected at step: " << iStep << " of " << (traj.rows() - 1)
                 << " substate: " << iSubStep << std::endl;

              ss << "     Names:";
              for (const auto& name : manip.getJointNames())
                ss << " " << name;

              ss << std::endl << "    State: " << subtraj.row(iSubStep) << std::endl;

              CONSOLE_BRIDGE_logError(ss.str().c_str());
            }
          }

          if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
            break;
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
      else
      {
        tesseract_common::TransformMap state = manip.calcFwdKin(traj.row(iStep));
        tesseract_collision::ContactResultMap sub_state_results =
            checkTrajectoryState(manager, state, config.contact_request);
        if (!sub_state_results.empty())
        {
          found = true;
          processInterpolatedSubSegmentCollisionResults(
              segment_results, sub_state_results, 0, 0, manager.getActiveCollisionObjects(), true);

          if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
          {
            std::stringstream ss;
            ss << "Discrete collision detected at step: " << iStep << " of " << (traj.rows() - 1) << std::endl;

            ss << "     Names:";
            for (const auto& name : manip.getJointNames())
              ss << " " << name;

            ss << std::endl << "    State: " << traj.row(iStep) << std::endl;

            CONSOLE_BRIDGE_logError(ss.str().c_str());
          }
        }

        if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
          break;
      }
    }
  }
  else
  {
    for (int iStep = 0; iStep < traj.rows(); ++iStep)
    {
      tesseract_collision::ContactResultMap& state_results = contacts[static_cast<size_t>(iStep)];
      state_results.clear();

      tesseract_common::TransformMap state = manip.calcFwdKin(traj.row(iStep));
      tesseract_collision::ContactResultMap sub_state_results =
          checkTrajectoryState(manager, state, config.contact_request);
      if (!sub_state_results.empty())
      {
        found = true;
        processInterpolatedSubSegmentCollisionResults(
            state_results, sub_state_results, 0, 0, manager.getActiveCollisionObjects(), true);

        if (console_bridge::getLogLevel() > console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        {
          std::stringstream ss;
          ss << "Discrete collision detected at step: " << iStep << " of " << (traj.rows() - 1) << std::endl;

          ss << "     Names:";
          for (const auto& name : manip.getJointNames())
            ss << " " << name;

          ss << std::endl << "    State0: " << traj.row(iStep) << std::endl;

          CONSOLE_BRIDGE_logError(ss.str().c_str());
        }
      }

      if (found && (config.contact_request.type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }
  return found;
}

}  // namespace tesseract_environment
