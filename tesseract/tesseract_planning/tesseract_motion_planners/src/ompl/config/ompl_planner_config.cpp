/**
 * @file ompl_planner_config.cpp
 * @brief Tesseract OMPL planner config implementation.
 *
 * @author Levi Armstrong
 * @date January 22, 2020
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

#include <tesseract_motion_planners/ompl/config/ompl_planner_config.h>
#include <tesseract/tesseract.h>

namespace tesseract_motion_planners
{
OMPLPlannerConfig::OMPLPlannerConfig(tesseract::Tesseract::ConstPtr tesseract,
                                     std::string manipulator,
                                     std::vector<OMPLPlannerConfigurator::ConstPtr> planners)
  : tesseract(std::move(tesseract)), manipulator(std::move(manipulator)), planners(std::move(planners))
{
}

OMPLPlannerConfig::OMPLPlannerConfig(tesseract::Tesseract::ConstPtr tesseract,
                                     std::string manipulator,
                                     std::vector<OMPLPlannerConfigurator::ConstPtr> planners,
                                     ompl::geometric::SimpleSetupPtr simple_setup)
  : simple_setup(std::move(simple_setup))
  , tesseract(std::move(tesseract))
  , manipulator(std::move(manipulator))
  , planners(std::move(planners))
{
}

bool OMPLPlannerConfig::generate()
{
  return ((simple_setup != nullptr) && (tesseract != nullptr) && (!manipulator.empty()) && (!planners.empty()));
}

tesseract_common::TrajArray OMPLPlannerConfig::getTrajectory() const
{
  const auto& path = this->simple_setup->getSolutionPath();
  const auto n_points = static_cast<long>(path.getStateCount());
  const auto dof = static_cast<long>(path.getSpaceInformation()->getStateDimension());

  tesseract_common::TrajArray result(n_points, dof);
  std::vector<double> state(static_cast<std::size_t>(dof));
  for (long i = 0; i < n_points; ++i)
  {
    simple_setup->getStateSpace()->copyToReals(state, path.getState(static_cast<unsigned>(i)));
    result.row(i) = Eigen::Map<Eigen::VectorXd>(state.data(), dof);
  }

  return result;
}

}  // namespace tesseract_motion_planners
