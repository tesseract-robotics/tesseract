/**
 * @file utils/utils.h
 * @brief General utilities and all inclusive utility header
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_UTILS_UTILS_H
#define TESSERACT_COMMAND_LANGUAGE_UTILS_UTILS_H

#include <tesseract_command_language/composite_instruction.h>

#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>

namespace tesseract_planning
{
/** @brief Gets joint position from waypoints that contain that information. Throws if waypoint does not directly
 * contain that information*/
const Eigen::VectorXd& getJointPosition(const Waypoint& waypoint);

/**
 * @brief This creates a seed by looping over and replacing every plan instruction with a composite instruction
 * @param instructions
 * @return
 */
CompositeInstruction generateSkeletonSeed(const CompositeInstruction& composite_instructions);

/**
 * @brief This loops over the instructions validates the structure
 *
 * Every plan instruction in /p composite_instruction should have a cooresponding CompositeInstruction
 *
 * @param composite_instructions
 * @param composite_seed
 * @return
 */
bool validateSeedStructure(const CompositeInstruction& composite_instructions,
                           const CompositeInstruction& composite_seed);

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_COMMAND_LANGUAGE_UTILS_H
