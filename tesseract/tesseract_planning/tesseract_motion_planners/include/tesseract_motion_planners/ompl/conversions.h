/**
 * @file conversions.h
 * @brief Tesseract OMPL planner conversions
 *
 * @author Jonathan Meyer
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_CONVERSIONS_H
#define TESSERACT_MOTION_PLANNERS_OMPL_CONVERSIONS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/PathGeometric.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_motion_planners
{
/**
 * @brief Convert an ompl path to tesseract TrajArray
 * @param path OMPL Path
 * @return Tesseract TrajArray
 */
tesseract_common::TrajArray toTrajArray(const ompl::geometric::PathGeometric& path);
}  // namespace tesseract_motion_planners

#endif
