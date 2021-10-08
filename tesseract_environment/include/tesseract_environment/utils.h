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
#ifndef TESSERACT_ENVIRONMENT_CORE_UTILS_H
#define TESSERACT_ENVIRONMENT_CORE_UTILS_H

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_environment/environment.h>

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
                                 bool active);

/**
 * @brief Should perform a continuous collision check between two states.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state0 First environment state
 * @param state1 Second environment state
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool checkTrajectorySegment(std::vector<tesseract_collision::ContactResultMap>& contacts,
                            tesseract_collision::ContinuousContactManager& manager,
                            const tesseract_common::TransformMap& state0,
                            const tesseract_common::TransformMap& state1,
                            const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a discrete collision check a state.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A discrete contact manager
 * @param state First environment state
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool checkTrajectoryState(std::vector<tesseract_collision::ContactResultMap>& contacts,
                          tesseract_collision::DiscreteContactManager& manager,
                          const tesseract_common::TransformMap& state,
                          const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a continuous collision check over the trajectory and stop on first collision.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param joint_names JointNames corresponding to the values in traj (must be in same order)
 * @param traj The joint values at each time step
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::ContinuousContactManager& manager,
                     const tesseract_scene_graph::StateSolver& state_solver,
                     const std::vector<std::string>& joint_names,
                     const tesseract_common::TrajArray& traj,
                     const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a continuous collision check over the trajectory and stop on first collision.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param manip The kinematic joint group
 * @param traj The joint values at each time step
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::ContinuousContactManager& manager,
                     const tesseract_kinematics::JointGroup& manip,
                     const tesseract_common::TrajArray& traj,
                     const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a discrete collision check over the trajectory and stop on first collision.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param joint_names JointNames corresponding to the values in traj (must be in same order)
 * @param traj The joint values at each time step
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::DiscreteContactManager& manager,
                     const tesseract_scene_graph::StateSolver& state_solver,
                     const std::vector<std::string>& joint_names,
                     const tesseract_common::TrajArray& traj,
                     const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a discrete collision check over the trajectory and stop on first collision.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param manip The kinematic joint group
 * @param traj The joint values at each time step
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool checkTrajectory(std::vector<tesseract_collision::ContactResultMap>& contacts,
                     tesseract_collision::DiscreteContactManager& manager,
                     const tesseract_kinematics::JointGroup& manip,
                     const tesseract_common::TrajArray& traj,
                     const tesseract_collision::CollisionCheckConfig& config);

}  // namespace tesseract_environment
#endif  // TESSERACT_ENVIRONMENT_CORE_UTILS_H
