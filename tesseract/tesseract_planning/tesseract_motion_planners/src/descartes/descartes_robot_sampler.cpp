/**
 * @file descartes_robot_sampler.cpp
 * @brief Tesseract Descartes Robot Kinematics Sampler
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
#include <tesseract_motion_planners/descartes/impl/descartes_robot_sampler.hpp>

namespace tesseract_motion_planners
{
// Explicit template instantiation
template class DescartesRobotSampler<float>;
template class DescartesRobotSampler<double>;

}  // namespace tesseract_motion_planners
