/**
 * @file ompl_profile.h
 * @brief Tesseract OMPL profile
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROFILE_H

#include <vector>
#include <memory>
#include <tesseract_command_language/plan_instruction.h>

#include <tesseract_motion_planners/ompl/ompl_problem.h>

namespace tesseract_planning
{

class OMPLPlanProfile
{
public:
  using Ptr = std::shared_ptr<OMPLPlanProfile>;
  using ConstPtr = std::shared_ptr<const OMPLPlanProfile>;

  virtual void apply(OMPLProblem& prob,
                     const Eigen::Isometry3d& cartesian_waypoint,
                     const PlanInstruction& parent_instruction,
                     const std::vector<std::string> &active_links,
                     int index) = 0;

  virtual void apply(OMPLProblem& prob,
                     const Eigen::VectorXd& joint_waypoint,
                     const PlanInstruction& parent_instruction,
                     const std::vector<std::string> &active_links,
                     int index) = 0;
};

/** @todo Currently OMPL does not have support of composite profile everything is handled by the plan profile */

}

#endif // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_PROFILE_H
