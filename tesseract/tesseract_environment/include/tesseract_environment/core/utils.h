/**
 * @file utils.h
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
#ifndef TESSERACT_ENVIRONMENT_UTILS_H
#define TESSERACT_ENVIRONMENT_UTILS_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <utility>
#include <unordered_map>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_environment/core/environment.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/types.h>

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
inline void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                        const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
                                        const std::string& current_link,
                                        bool active)
{
  // recursively get all active links
  assert(scene_graph->isTree());
  if (active)
  {
    active_links.push_back(current_link);
    for (const auto& child_link : scene_graph->getAdjacentLinkNames(current_link))
      getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
  else
  {
    for (const auto& child_link : scene_graph->getAdjacentLinkNames(current_link))
      if (scene_graph->getInboundJoints(child_link)[0]->type != tesseract_scene_graph::JointType::FIXED)
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, true);
      else
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
  }
}

/**
 * @brief Should perform a continuous collision check between two states.
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state0 First environment state
 * @param state1 Second environment state
 * @param type The type of contact test to perform (FIRST, CLOSEST, ALL)
 * @param verbose Print out found collisions
 * @return True if collision was found, otherwise false.
 */
inline bool
checkTrajectorySegment(std::vector<tesseract_collision::ContactResultMap>& contacts,
                       tesseract_collision::ContinuousContactManager& manager,
                       const tesseract_environment::EnvState::Ptr& state0,
                       const tesseract_environment::EnvState::Ptr& state1,
                       tesseract_collision::ContactTestType type = tesseract_collision::ContactTestType::FIRST,
                       bool verbose = false)
{
  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

  tesseract_collision::ContactResultMap collisions;
  manager.contactTest(collisions, type);

  if (!collisions.empty())
  {
    if (verbose)
    {
      for (auto& collision : collisions)
      {
        std::stringstream ss;
        ss << "Continuous collision detected between '" << collision.first.first << "' and '" << collision.first.second
           << "' with distance " << collision.second.front().distance << std::endl;

        CONSOLE_BRIDGE_logError(ss.str().c_str());
      }
    }

    contacts.push_back(collisions);
    return true;
  }

  return false;
}

/**
 * @brief Should perform a discrete collision check a state.
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A discrete contact manager
 * @param state First environment state
 * @param type The type of contact test to perform (FIRST, CLOSEST, ALL)
 * @param verbose Print out found collisions
 * @return True if collision was found, otherwise false.
 */
inline bool
checkTrajectoryState(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::DiscreteContactManager& manager,
                     const tesseract_environment::EnvState::Ptr& state,
                     tesseract_collision::ContactTestType type = tesseract_collision::ContactTestType::FIRST,
                     bool verbose = false)
{
  tesseract_collision::ContactResultMap collisions;

  for (const auto& link_name : manager.getActiveCollisionObjects())
    manager.setCollisionObjectsTransform(link_name, state->transforms[link_name]);

  manager.contactTest(collisions, type);

  if (!collisions.empty())
  {
    if (verbose)
    {
      for (auto& collision : collisions)
      {
        std::stringstream ss;
        ss << "Discrete collision detected between '" << collision.first.first << "' and '" << collision.first.second
           << "' with distance " << collision.second.front().distance << std::endl;

        CONSOLE_BRIDGE_logError(ss.str().c_str());
      }
    }

    contacts.push_back(collisions);
    return true;
  }

  return false;
}

/**
 * @brief Should perform a continuous collision check over the trajectory and stop on first collision.
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param joint_names JointNames corresponding to the values in traj (must be in same order)
 * @param traj The joint values at each time step
 * @param type The type of contact test to perform (FIRST, CLOSEST, ALL)
 * @return True if collision was found, otherwise false.
 */
inline bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                            tesseract_collision::ContinuousContactManager& manager,
                            const tesseract_environment::StateSolver& state_solver,
                            const std::vector<std::string>& joint_names,
                            const tesseract_common::TrajArray& traj,
                            tesseract_collision::ContactTestType type = tesseract_collision::ContactTestType::FIRST,
                            bool verbose = false)
{
  bool found = false;

  contacts.reserve(static_cast<size_t>(traj.rows() - 1));
  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
  {
    tesseract_environment::EnvState::Ptr state0 = state_solver.getState(joint_names, traj.row(iStep));
    tesseract_environment::EnvState::Ptr state1 = state_solver.getState(joint_names, traj.row(iStep + 1));

    if (checkTrajectorySegment(contacts, manager, state0, state1, type, verbose))
    {
      found = true;
      if (verbose)
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

    if (found && (type == tesseract_collision::ContactTestType::FIRST))
      break;
  }

  return found;
}

/**
 * @brief Should perform a continuous collision check over the trajectory and stop on first collision.
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param joint_names JointNames corresponding to the values in traj (must be in same order)
 * @param traj The joint values at each time step
 * @param longest_valid_segment_length Used to check collisions between two state if norm(state0-state1) >
 * longest_valid_segment_length.
 * @param type The type of contact test to perform (FIRST, CLOSEST, ALL)
 * @return True if collision was found, otherwise false.
 */
inline bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                            tesseract_collision::ContinuousContactManager& manager,
                            const tesseract_environment::StateSolver& state_solver,
                            const std::vector<std::string>& joint_names,
                            const tesseract_common::TrajArray& traj,
                            double longest_valid_segment_length,
                            tesseract_collision::ContactTestType type = tesseract_collision::ContactTestType::FIRST,
                            bool verbose = false)
{
  bool found = false;

  contacts.reserve(static_cast<size_t>(traj.rows() - 1));
  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
  {
    double dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();
    if (dist > longest_valid_segment_length)
    {
      long cnt = static_cast<long>(std::ceil(dist / longest_valid_segment_length)) + 1;
      tesseract_common::TrajArray subtraj(cnt, traj.cols());
      for (long iVar = 0; iVar < traj.cols(); ++iVar)
        subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

      for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
      {
        tesseract_environment::EnvState::Ptr state0 = state_solver.getState(joint_names, subtraj.row(iSubStep));
        tesseract_environment::EnvState::Ptr state1 = state_solver.getState(joint_names, subtraj.row(iSubStep + 1));
        if (checkTrajectorySegment(contacts, manager, state0, state1, type, verbose))
        {
          found = true;
          if (verbose)
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

        if (found && (type == tesseract_collision::ContactTestType::FIRST))
          break;
      }

      if (found && (type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
    else
    {
      tesseract_environment::EnvState::Ptr state0 = state_solver.getState(joint_names, traj.row(iStep));
      tesseract_environment::EnvState::Ptr state1 = state_solver.getState(joint_names, traj.row(iStep + 1));
      if (checkTrajectorySegment(contacts, manager, state0, state1, type, verbose))
      {
        found = true;
        if (verbose)
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

      if (found && (type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  return found;
}

/**
 * @brief Should perform a discrete collision check over the trajectory and stop on first collision.
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param joint_names JointNames corresponding to the values in traj (must be in same order)
 * @param traj The joint values at each time step
 * @param type The type of contact test to perform (FIRST, CLOSEST, ALL)
 * @return True if collision was found, otherwise false.
 */
inline bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                            tesseract_collision::DiscreteContactManager& manager,
                            const tesseract_environment::StateSolver& state_solver,
                            const std::vector<std::string>& joint_names,
                            const tesseract_common::TrajArray& traj,
                            tesseract_collision::ContactTestType type = tesseract_collision::ContactTestType::FIRST,
                            bool verbose = false)
{
  bool found = false;

  contacts.reserve(static_cast<size_t>(traj.rows()));
  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
  {
    tesseract_environment::EnvState::Ptr state = state_solver.getState(joint_names, traj.row(iStep));
    if (checkTrajectoryState(contacts, manager, state, type, verbose))
    {
      found = true;
      if (verbose)
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

    if (found && (type == tesseract_collision::ContactTestType::FIRST))
      break;
  }

  return found;
}

/**
 * @brief Should perform a discrete collision check over the trajectory and stop on first collision.
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param joint_names JointNames corresponding to the values in traj (must be in same order)
 * @param traj The joint values at each time step
 * @param longest_valid_segment_length Used to check collisions between two state if norm(state0-state1) >
 * longest_valid_segment_length.
 * @param type The type of contact test to perform (FIRST, CLOSEST, ALL)
 * @return True if collision was found, otherwise false.
 */
inline bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                            tesseract_collision::DiscreteContactManager& manager,
                            const tesseract_environment::StateSolver& state_solver,
                            const std::vector<std::string>& joint_names,
                            const tesseract_common::TrajArray& traj,
                            double longest_valid_segment_length,
                            tesseract_collision::ContactTestType type = tesseract_collision::ContactTestType::FIRST,
                            bool verbose = false)
{
  bool found = false;

  contacts.reserve(static_cast<size_t>(traj.rows()));
  for (int iStep = 0; iStep < traj.rows(); ++iStep)
  {
    double dist = -1;
    if (iStep < traj.rows() - 1)
      dist = (traj.row(iStep + 1) - traj.row(iStep)).norm();

    if (dist > 0 && dist > longest_valid_segment_length)
    {
      int cnt = static_cast<int>(std::ceil(dist / longest_valid_segment_length)) + 1;
      tesseract_common::TrajArray subtraj(cnt, traj.cols());
      for (long iVar = 0; iVar < traj.cols(); ++iVar)
        subtraj.col(iVar) = Eigen::VectorXd::LinSpaced(cnt, traj.row(iStep)(iVar), traj.row(iStep + 1)(iVar));

      for (int iSubStep = 0; iSubStep < subtraj.rows() - 1; ++iSubStep)
      {
        tesseract_environment::EnvState::Ptr state = state_solver.getState(joint_names, subtraj.row(iSubStep));
        if (checkTrajectoryState(contacts, manager, state, type, verbose))
        {
          found = true;
          if (verbose)
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

        if (found && (type == tesseract_collision::ContactTestType::FIRST))
          break;
      }

      if (found && (type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
    else
    {
      tesseract_environment::EnvState::Ptr state = state_solver.getState(joint_names, traj.row(iStep));
      if (checkTrajectoryState(contacts, manager, state, type, verbose))
      {
        found = true;
        if (verbose)
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

      if (found && (type == tesseract_collision::ContactTestType::FIRST))
        break;
    }
  }

  return found;
}
}  // namespace tesseract_environment
#endif  // TESSERACT_ENVIRONMENT_UTILS_H
