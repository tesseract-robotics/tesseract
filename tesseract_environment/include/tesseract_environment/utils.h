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
 * @brief Should perform a continuous collision check between two states configuring the manager with the config
 * @param manager A continuous contact manager
 * @param state0 First environment state
 * @param state1 Second environment state
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return Return the contact results map. If empty no contacts were found
 */
tesseract_collision::ContactResultMap checkTrajectorySegment(tesseract_collision::ContinuousContactManager& manager,
                                                             const tesseract_common::TransformMap& state0,
                                                             const tesseract_common::TransformMap& state1,
                                                             const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a continuous collision check between two states only passing along the contact_request to the
 * manager
 * @param manager A continuous contact manager
 * @param state0 First environment state
 * @param state1 Second environment state
 * @param contact_request Contact request passed to the manager
 * @return Return the contact results map. If empty not contacts were found
 */
tesseract_collision::ContactResultMap
checkTrajectorySegment(tesseract_collision::ContinuousContactManager& manager,
                       const tesseract_common::TransformMap& state0,
                       const tesseract_common::TransformMap& state1,
                       const tesseract_collision::ContactRequest& contact_request);

/**
 * @brief Should perform a discrete collision check a state first configuring manager with config
 * @param manager A discrete contact manager
 * @param state First environment state
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return Return the contact results map. If empty no contacts were found
 */
tesseract_collision::ContactResultMap checkTrajectoryState(tesseract_collision::DiscreteContactManager& manager,
                                                           const tesseract_common::TransformMap& state,
                                                           const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a discrete collision check a state only passing contact_request to the manager
 * @param manager A discrete contact manager
 * @param state First environment state
 * @param contact_request Contact request passed to the manager
 * @return Return the contact results map. If empty no contacts were found
 */
tesseract_collision::ContactResultMap checkTrajectoryState(tesseract_collision::DiscreteContactManager& manager,
                                                           const tesseract_common::TransformMap& state,
                                                           const tesseract_collision::ContactRequest& contact_request);

/**
 * @brief This processes interpolated contact results and updated cc_time and cc_type
 * @details This is copied from the trajopt utility processInterpolatedCollisionResults
 * @param segment_results The full results to store sub segment results in after processing
 * @param sub_segment_results The interpolated results to process
 * @param sub_segment_index The current sub segment index
 * @param sub_segment_last_index The last sub segment index
 * @param manip_active_link_names The active link names
 * @param discrete If discrete contact checker was used
 */
void processInterpolatedSubSegmentCollisionResults(tesseract_collision::ContactResultMap& segment_results,
                                                   tesseract_collision::ContactResultMap& sub_segment_results,
                                                   int sub_segment_index,
                                                   int sub_segment_last_index,
                                                   const std::vector<std::string>& manip_active_link_names,
                                                   bool discrete);

/**
 * @brief Should perform a continuous collision check over the trajectory and stop on first collision.
 * @param contacts A vector of ContactMap where each index corresponds to a segment in the trajectory. The length should
 * be trajectory size minus one.
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
 * @param contacts A vector of ContactMap where each index corresponds to a segment in the trajectory. The length should
 * be trajectory size minus one.
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
 * @param contacts A vector of ContactMap where each index corresponds to a segment in the trajectory, except the last
 * which is the end state. The length should be the same size as the input trajectory.
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
 * @param contacts A vector of ContactMap where each index corresponds to a segment in the trajectory, except the last
 * which is the end state. The length should be the same size as the input trajectory.
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
