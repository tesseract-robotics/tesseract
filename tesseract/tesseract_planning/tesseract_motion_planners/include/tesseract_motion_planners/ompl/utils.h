/**
 * @file utils.h
 * @brief Tesseract OMPL planner utility functions
 *
 * @author Levi Armstrong
 * @date February 17, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H
#define TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <Eigen/Geometry>
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

namespace tesseract_motion_planners
{
using OMPLStateExtractor = std::function<Eigen::Map<Eigen::VectorXd>(const ompl::base::State*)>;

Eigen::Map<Eigen::VectorXd> RealVectorStateSpaceExtractor(const ompl::base::State* s1, unsigned dimension);

#ifndef OMPL_LESS_1_4_0
Eigen::Map<Eigen::VectorXd> ConstrainedStateSpaceExtractor(const ompl::base::State* s1);
#endif

/**
 * @brief Convert an ompl path to tesseract TrajArray
 * @param path OMPL Path
 * @param extractor This function understands the type of state space and converts it to an eigen vector.
 * @return Tesseract TrajArray
 */
tesseract_common::TrajArray toTrajArray(const ompl::geometric::PathGeometric& path, OMPLStateExtractor extractor);
}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_OMPL_UTILS_H
