/**
 * @file lvs_interpolation.h
 * @brief Provides interpolation functions apply longest valid segment logic
 *
 * @author Matthew Powelson
 * @date July 23, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_LVS_INTERPOLATION_H
#define TESSERACT_MOTION_PLANNERS_LVS_INTERPOLATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/command_language.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/visibility_control.h>

namespace tesseract_planning
{
/**
 * @brief LVSInterpolateStateWaypoint(JointWaypoint to JointWaypoint)
 *
 * This function interpolates the motion from start state to end state. Results are stored in StateWaypoint objects.
 *
 * CASE 1: The base_instruction type is FREESPACE:
 * - the interpolation will be done in joint space
 * - the number of steps for the plan will be calculated such that the norm of all joint distances between successive
 *   steps is no longer than state_longest_valid_segment_length
 *
 * CASE 2: The base_instrucation type is LINEAR:
 * - the interpolation will be done in cartesian space
 * - the number of steps for the plan will be calculated such that:
 *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *
 * @param start The joint state at the start of the plan
 * @param end The joint state at the end of the plan
 * @param base_instruction The base plan instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param state_longest_valid_segment_length The maximum joint distance (norm of changes to all joint positions) between
 *successive steps
 * @param translation_longest_valid_segment_length The maximum translational distance between successive steps
 * @param rotation_longest_valid_segment_length The maximaum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @return A composite instruction of move instruction with state waypoints
 **/
TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC CompositeInstruction
LVSInterpolateStateWaypoint(const JointWaypoint& start,
                            const JointWaypoint& end,
                            const PlanInstruction& base_instruction,
                            const PlannerRequest& request,
                            const ManipulatorInfo& manip_info,
                            double state_longest_valid_segment_length,
                            double translation_longest_valid_segment_length,
                            double rotation_longest_valid_segment_length,
                            int min_steps);

/**
 * @brief LVSInterpolateStateWaypoint(JointWaypoint to CartesianWaypoint)
 *
 * This will interpolate joint positions from the start state to the end state, where the norm of all joint movements
 * at each step is no longer than state_longest_valid_segment_length.
 *
 * @param start The joint state at the start of the plan
 * @param end The cartesian state at the end of the plan
 * @param base_instruction The base plan instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param state_longest_valid_segment_length The longest valid joint distance (norm of all joint movements) per step
 * @param min_steps The minimum number of steps for the plan
 * @return A composite instruction of move instruction with state waypoints
 **/
TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC CompositeInstruction
LVSInterpolateStateWaypoint(const JointWaypoint& start,
                            const CartesianWaypoint& end,
                            const PlanInstruction& base_instruction,
                            const PlannerRequest& request,
                            const ManipulatorInfo& manip_info,
                            double state_longest_valid_segment_length,
                            double translation_longest_valid_segment_length,
                            double rotation_longest_valid_segment_length,
                            int min_steps);

/**
 * @brief LVSInterpolateStateWaypoint(JointWaypoint to JointWaypoint)
 *
 * This function interpolates the motion from start state to end state. Results are stored in StateWaypoint objects.
 *
 * CASE 1: The base_instruction type is FREESPACE:
 * - the interpolation will be done in joint space
 * - the number of steps for the plan will be calculated such that the norm of all joint distances between successive
 *   steps is no longer than state_longest_valid_segment_length
 *
 * CASE 2: The base_instrucation type is LINEAR:
 * - the interpolation will be done in cartesian space
 * - the number of steps for the plan will be calculated such that:
 *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *
 * @param start The joint state at the start of the plan
 * @param end The joint state at the end of the plan
 * @param base_instruction The base plan instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param state_longest_valid_segment_length The maximum joint distance (norm of changes to all joint positions) between
 *successive steps
 * @param translation_longest_valid_segment_length The maximum translational distance between successive steps
 * @param rotation_longest_valid_segment_length The maximaum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @return A composite instruction of move instruction with state waypoints
 **/
TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC CompositeInstruction
LVSInterpolateStateWaypoint(const CartesianWaypoint& start,
                            const JointWaypoint& end,
                            const PlanInstruction& base_instruction,
                            const PlannerRequest& request,
                            const ManipulatorInfo& manip_info,
                            double state_longest_valid_segment_length,
                            double translation_longest_valid_segment_length,
                            double rotation_longest_valid_segment_length,
                            int min_steps);

/**
 * @brief LVSInterpolateStateWaypoint(JointWaypoint to JointWaypoint)
 *
 * This function interpolates the motion from start state to end state. Results are stored in StateWaypoint objects.
 *
 * CASE 1: The base_instruction type is FREESPACE:
 * - the interpolation will be done in joint space
 * - the number of steps for the plan will be calculated such that the norm of all joint distances between successive
 *   steps is no longer than state_longest_valid_segment_length
 *
 * CASE 2: The base_instrucation type is LINEAR:
 * - the interpolation will be done in cartesian space
 * - the number of steps for the plan will be calculated such that:
 *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *
 * @param start The joint state at the start of the plan
 * @param end The joint state at the end of the plan
 * @param base_instruction The base plan instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param state_longest_valid_segment_length The maximum joint distance (norm of changes to all joint positions) between
 *successive steps
 * @param translation_longest_valid_segment_length The maximum translational distance between successive steps
 * @param rotation_longest_valid_segment_length The maximaum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @return A composite instruction of move instruction with state waypoints
 **/
TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC CompositeInstruction
LVSInterpolateStateWaypoint(const CartesianWaypoint& start,
                            const CartesianWaypoint& end,
                            const PlanInstruction& base_instruction,
                            const PlannerRequest& request,
                            const ManipulatorInfo& manip_info,
                            double state_longest_valid_segment_length,
                            double translation_longest_valid_segment_length,
                            double rotation_longest_valid_segment_length,
                            int min_steps);

/**
 * @brief LVSInterpolateCartStateWaypoint(JointWaypoint to JointWaypoint)
 *
 * This function interpolates the motion from start state to end state. Results are stored in CartesianWaypoint objects.
 *
 * CASE 1: The base_instruction type is FREESPACE:
 * - the interpolation will be done in joint space
 * - the number of steps for the plan will be calculated such that the norm of all joint distances between successive
 *   steps is no longer than state_longest_valid_segment_length
 *
 * CASE 2: The base_instrucation type is LINEAR:
 * - the interpolation will be done in cartesian space
 * - the number of steps for the plan will be calculated such that:
 *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *
 * @param start The joint state at the start of the plan
 * @param end The joint state at the end of the plan
 * @param base_instruction The base plan instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param state_longest_valid_segment_length The maximum joint distance (norm of changes to all joint positions) between
 *successive steps
 * @param translation_longest_valid_segment_length The maximum translational distance between successive steps
 * @param rotation_longest_valid_segment_length The maximaum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @return A composite instruction of move instruction with state waypoints
 **/
TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC CompositeInstruction
LVSInterpolateCartStateWaypoint(const JointWaypoint& start,
                                const JointWaypoint& end,
                                const PlanInstruction& base_instruction,
                                const PlannerRequest& request,
                                const ManipulatorInfo& manip_info,
                                double state_longest_valid_segment_length,
                                double translation_longest_valid_segment_length,
                                double rotation_longest_valid_segment_length,
                                int min_steps);

/**
 * @brief LVSInterpolateCartStateWaypoint(JointWaypoint to CartesianWaypoint)
 *
 * This function interpolates the motion from start state to end state. Results are stored in CartesianWaypoint objects.
 *
 * CASE 1: The base_instruction type is FREESPACE:
 * - the interpolation will be done in joint space
 * - the number of steps for the plan will be calculated such that the norm of all joint distances between successive
 *   steps is no longer than state_longest_valid_segment_length
 *
 * CASE 2: The base_instrucation type is LINEAR:
 * - the interpolation will be done in cartesian space
 * - the number of steps for the plan will be calculated such that:
 *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *
 * @param start The joint state at the start of the plan
 * @param end The joint state at the end of the plan
 * @param base_instruction The base plan instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param state_longest_valid_segment_length The maximum joint distance (norm of changes to all joint positions) between
 *successive steps
 * @param translation_longest_valid_segment_length The maximum translational distance between successive steps
 * @param rotation_longest_valid_segment_length The maximaum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @return A composite instruction of move instruction with state waypoints
 **/
TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC CompositeInstruction
LVSInterpolateCartStateWaypoint(const JointWaypoint& start,
                                const CartesianWaypoint& end,
                                const PlanInstruction& base_instruction,
                                const PlannerRequest& request,
                                const ManipulatorInfo& manip_info,
                                double state_longest_valid_segment_length,
                                double translation_longest_valid_segment_length,
                                double rotation_longest_valid_segment_length,
                                int min_steps);

/**
 * @brief LVSInterpolateCartStateWaypoint(CartesianWaypoint to JointWaypoint)
 *
 * This function interpolates the motion from start state to end state. Results are stored in CartesianWaypoint objects.
 *
 * CASE 1: The base_instruction type is FREESPACE:
 * - the interpolation will be done in joint space
 * - the number of steps for the plan will be calculated such that the norm of all joint distances between successive
 *   steps is no longer than state_longest_valid_segment_length
 *
 * CASE 2: The base_instrucation type is LINEAR:
 * - the interpolation will be done in cartesian space
 * - the number of steps for the plan will be calculated such that:
 *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *
 * @param start The joint state at the start of the plan
 * @param end The joint state at the end of the plan
 * @param base_instruction The base plan instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param state_longest_valid_segment_length The maximum joint distance (norm of changes to all joint positions) between
 *successive steps
 * @param translation_longest_valid_segment_length The maximum translational distance between successive steps
 * @param rotation_longest_valid_segment_length The maximaum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @return A composite instruction of move instruction with state waypoints
 **/
TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC CompositeInstruction
LVSInterpolateCartStateWaypoint(const CartesianWaypoint& start,
                                const JointWaypoint& end,
                                const PlanInstruction& base_instruction,
                                const PlannerRequest& request,
                                const ManipulatorInfo& manip_info,
                                double state_longest_valid_segment_length,
                                double translation_longest_valid_segment_length,
                                double rotation_longest_valid_segment_length,
                                int min_steps);

/**
 * @brief LVSInterpolateCartStateWaypoint(CartesianWaypoint to CartesianWaypoint)
 *
 * This function interpolates the motion from start state to end state. Results are stored in CartesianWaypoint objects.
 *
 * CASE 1: The base_instruction type is FREESPACE:
 * - the interpolation will be done in joint space
 * - the number of steps for the plan will be calculated such that the norm of all joint distances between successive
 *   steps is no longer than state_longest_valid_segment_length
 *
 * CASE 2: The base_instrucation type is LINEAR:
 * - the interpolation will be done in cartesian space
 * - the number of steps for the plan will be calculated such that:
 *   - the translational distance between successive steps is no longer than translation_longest_valid_segment
 *   - the rotational distance between successive steps is no longer than rotation_longest_valid_segment
 *
 * @param start The joint state at the start of the plan
 * @param end The joint state at the end of the plan
 * @param base_instruction The base plan instruction
 * @param request The planning request information
 * @param manip_info The manipulator information provided by the parent composite instruction
 * @param state_longest_valid_segment_length The maximum joint distance (norm of changes to all joint positions) between
 *successive steps
 * @param translation_longest_valid_segment_length The maximum translational distance between successive steps
 * @param rotation_longest_valid_segment_length The maximaum rotational distance between successive steps
 * @param min_steps The minimum number of steps for the plan
 * @return A composite instruction of move instruction with state waypoints
 **/
TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC CompositeInstruction
LVSInterpolateCartStateWaypoint(const CartesianWaypoint& start,
                                const CartesianWaypoint& end,
                                const PlanInstruction& base_instruction,
                                const PlannerRequest& request,
                                const ManipulatorInfo& manip_info,
                                double state_longest_valid_segment_length,
                                double translation_longest_valid_segment_length,
                                double rotation_longest_valid_segment_length,
                                int min_steps);
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_H
