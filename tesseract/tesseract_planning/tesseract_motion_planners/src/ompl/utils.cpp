/**
 * @file utils.cpp
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <memory>

#ifndef OMPL_LESS_1_4_0
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#endif

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/utils.h>

namespace tesseract_motion_planners
{
Eigen::Map<Eigen::VectorXd> RealVectorStateSpaceExtractor(const ompl::base::State* s1, unsigned dimension)
{
  assert(dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*>(s1) != nullptr);
  const auto* s = s1->template as<ompl::base::RealVectorStateSpace::StateType>();
  return Eigen::Map<Eigen::VectorXd>(s->values, dimension);
}

#ifndef OMPL_LESS_1_4_0
Eigen::Map<Eigen::VectorXd> ConstrainedStateSpaceExtractor(const ompl::base::State* s1)
{
  assert(dynamic_cast<const ompl::base::ProjectedStateSpace::StateType*>(s1) != nullptr);
  const Eigen::Map<Eigen::VectorXd>& s = *(s1->template as<ompl::base::ProjectedStateSpace::StateType>());
  return s;
}
#endif

tesseract_common::TrajArray toTrajArray(const ompl::geometric::PathGeometric& path, OMPLStateExtractor extractor)
{
  const auto n_points = static_cast<long>(path.getStateCount());
  const auto dof = static_cast<long>(path.getSpaceInformation()->getStateDimension());

  tesseract_common::TrajArray result(n_points, dof);
  for (long i = 0; i < n_points; ++i)
    result.row(i) = extractor(path.getState(static_cast<unsigned>(i)));

  return result;
}

}  // namespace tesseract_motion_planners
