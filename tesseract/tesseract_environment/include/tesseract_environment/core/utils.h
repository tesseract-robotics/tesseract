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
                                          const tesseract_scene_graph::SceneGraphConstPtr& scene_graph,
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
   * @brief Should perform a continuous collision check over the trajectory and stop on first collision.
   * @param manager A continuous contact manager
   * @param env The environment
   * @param joint_names JointNames corresponding to the values in traj (must be in same order)
   * @param traj The joint values at each time step
   * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
   * @param first_only Indicates if it should return on first contact
   * @return True if collision was found, otherwise false.
   */
  inline bool checkTrajectory(tesseract_collision::ContinuousContactManager& manager,
                              const tesseract_environment::Environment& env,
                              const std::vector<std::string>& joint_names,
                              const tesseract_common::TrajArray& traj,
                              std::vector<tesseract_collision::ContactResultMap>& contacts,
                              bool first_only = true)
  {
    bool found = false;

    contacts.reserve(static_cast<size_t>(traj.rows() - 1));
    for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
    {
      tesseract_collision::ContactResultMap collisions;

      tesseract_environment::EnvStatePtr state0 = env.getState(joint_names, traj.row(iStep));
      tesseract_environment::EnvStatePtr state1 = env.getState(joint_names, traj.row(iStep + 1));

      for (const auto& link_name : manager.getActiveCollisionObjects())
        manager.setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

      manager.contactTest(collisions, tesseract_collision::ContactTestTypes::FIRST);

      if (collisions.size() > 0)
      {
        found = true;
        contacts.push_back(collisions);
      }

      if (found && first_only)
        break;
    }

    return found;
  }

  /**
   * @brief Should perform a discrete collision check over the trajectory and stop on first collision.
   * @param manager A continuous contact manager
   * @param env The environment
   * @param joint_names JointNames corresponding to the values in traj (must be in same order)
   * @param traj The joint values at each time step
   * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
   * @param first_only Indicates if it should return on first contact
   * @return True if collision was found, otherwise false.
   */
  inline bool checkTrajectory(tesseract_collision::DiscreteContactManager& manager,
                              const tesseract_environment::Environment& env,
                              const std::vector<std::string>& joint_names,
                              const tesseract_common::TrajArray& traj,
                              std::vector<tesseract_collision::ContactResultMap>& contacts,
                              bool first_only = true)
  {
    bool found = false;

    contacts.reserve(static_cast<size_t>(traj.rows()));
    for (int iStep = 0; iStep < traj.rows(); ++iStep)
    {
      tesseract_collision::ContactResultMap collisions;

      tesseract_environment::EnvStatePtr state0 = env.getState(joint_names, traj.row(iStep));

      for (const auto& link_name : manager.getActiveCollisionObjects())
        manager.setCollisionObjectsTransform(link_name, state0->transforms[link_name]);

      manager.contactTest(collisions, tesseract_collision::ContactTestTypes::FIRST);

      if (collisions.size() > 0)
      {
        found = true;
        contacts.push_back(collisions);
      }
      if (found && first_only)
        break;
    }

    return found;
  }
  }    // namespace tesseract_environment
#endif // TESSERACT_ENVIRONMENT_UTILS_H
