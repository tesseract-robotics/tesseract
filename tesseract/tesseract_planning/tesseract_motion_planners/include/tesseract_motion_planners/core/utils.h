/**
 * @file utils.h
 * @brief Planner utility functions.
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
#ifndef TESSERACT_MOTION_PLANNERS_UTILS_H
#define TESSERACT_MOTION_PLANNERS_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
/**
 * @brief Inerpolate between two transforms return a vector of Eigen::Isometry transforms.
 * @param start The Start Transform
 * @param stop The Stop/End Transform
 * @param steps The number of step
 * @return A vector of Eigen::Isometry with a length = steps + 1
 */
tesseract_common::VectorIsometry3d interpolate(const Eigen::Isometry3d& start,
                                               const Eigen::Isometry3d& stop,
                                               int steps);

/**
 * @brief Inerpolate between two Eigen::VectorXd and return a Matrix
 * @param start The Start State
 * @param stop The Stop/End State
 * @param steps The number of step
 * @return A matrix where columns = steps + 1
 */
Eigen::MatrixXd interpolate(const Eigen::Ref<const Eigen::VectorXd>& start,
                            const Eigen::Ref<const Eigen::VectorXd>& stop,
                            int steps);

/**
 * @brief Inerpolate between two waypoints return a vector of waypoints.
 * @param start The Start Waypoint
 * @param stop The Stop/End Waypoint
 * @param steps The number of step
 * @return A vector of waypoints with a length = steps + 1
 */
std::vector<Waypoint> interpolate_waypoint(const Waypoint& start, const Waypoint& stop, int steps);

/**
 * @brief A program flatten filter
 * @param instruction The instruction to flatten
 * @param composite The parent composite the instruction is associated with
 * @param parent_is_first_composite Indicate if this is the top most composite
 * @return True if successful, otherwise false
 */
bool programFlattenFilter(const Instruction& instruction,
                          const CompositeInstruction& composite,
                          bool parent_is_first_composite);

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * @param composite_instruction Input composite instruction to be flattened
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<Instruction>> flattenProgram(CompositeInstruction& composite_instruction);

/**
 * @brief Flattens a CompositeInstruction into a vector of Instruction&
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * @param composite_instruction Input composite instruction to be flattened
 * @return A new flattened vector (const) referencing the original instruction elements
 */
std::vector<std::reference_wrapper<const Instruction>>
flattenProgram(const CompositeInstruction& composite_instruction);

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to composites
 * in the parent instruction
 *
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @return A new flattened vector referencing the original instruction elements
 */
std::vector<std::reference_wrapper<Instruction>> flattenProgramToPattern(CompositeInstruction& composite_instruction,
                                                                         const CompositeInstruction& pattern);

/**
 * @brief Flattens a composite instruction to the same pattern as the pattern composite instruction. ie, an element of
 * instruction will only be flattened if the corresponding element in pattern is flattenable.
 *
 * If /p composite_instruction parameter has a start instruction it is added but child composites are not check for
 * start instructions.
 *
 * The motivation for this utility is a case where you flatten only the elements in a seed that correspond to composites
 * in the parent instruction
 *
 * @param instruction CompositeInstruction that will be flattened
 * @param pattern CompositeInstruction used to determine if instruction will be flattened
 * @return A new flattened vector (const) referencing the original instruction elements
 */
std::vector<std::reference_wrapper<const Instruction>>
flattenProgramToPattern(const CompositeInstruction& composite_instruction, const CompositeInstruction& pattern);

/**
 * @brief Should perform a continuous collision check over the trajectory.
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::ContinuousContactManager& manager,
                         const tesseract_environment::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::CollisionCheckConfig& config);

/**
 * @brief Should perform a discrete collision check over the trajectory
 * @param contacts A vector of vector of ContactMap where each index corresponds to a timestep
 * @param manager A continuous contact manager
 * @param state_solver The environment state solver
 * @param program The program to check for contacts
 * @param config CollisionCheckConfig used to specify collision check settings
 * @return True if collision was found, otherwise false.
 */
bool contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                         tesseract_collision::DiscreteContactManager& manager,
                         const tesseract_environment::StateSolver& state_solver,
                         const CompositeInstruction& program,
                         const tesseract_collision::CollisionCheckConfig& config);

#ifndef SWIG

/// Deprecated overloads
bool DEPRECATED("Please use overload with CollisionCheckConfig")
    contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                        tesseract_collision::ContinuousContactManager& manager,
                        const tesseract_environment::StateSolver& state_solver,
                        const CompositeInstruction& program,
                        const tesseract_collision::ContactRequest& request =
                            tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::FIRST));

bool DEPRECATED("Please use overload with CollisionCheckConfig")
    contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                        tesseract_collision::ContinuousContactManager& manager,
                        const tesseract_environment::StateSolver& state_solver,
                        const CompositeInstruction& program,
                        double longest_valid_segment_length,
                        const tesseract_collision::ContactRequest& request =
                            tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::FIRST));

bool DEPRECATED("Please use overload with CollisionCheckConfig")
    contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                        tesseract_collision::DiscreteContactManager& manager,
                        const tesseract_environment::StateSolver& state_solver,
                        const CompositeInstruction& program,
                        const tesseract_collision::ContactRequest& request =
                            tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::FIRST));

bool DEPRECATED("Please use overload with CollisionCheckConfig")
    contactCheckProgram(std::vector<tesseract_collision::ContactResultMap>& contacts,
                        tesseract_collision::DiscreteContactManager& manager,
                        const tesseract_environment::StateSolver& state_solver,
                        const CompositeInstruction& program,
                        double longest_valid_segment_length,
                        const tesseract_collision::ContactRequest& request =
                            tesseract_collision::ContactRequest(tesseract_collision::ContactTestType::FIRST));

#endif  // SWIG

/**
 * @brief This generates a naive seed for the provided program
 * @details This will generate a seed where each plan instruction has a single move instruction associated to it using
 * the current state.
 * @param composite_instructions The input program
 * @param env The environment information
 * @return The generated seed
 */
CompositeInstruction generateNaiveSeed(const CompositeInstruction& composite_instructions,
                                       const tesseract_environment::Environment& env);

}  // namespace tesseract_planning

#endif  // TESSERACT_PLANNING_UTILS_H
