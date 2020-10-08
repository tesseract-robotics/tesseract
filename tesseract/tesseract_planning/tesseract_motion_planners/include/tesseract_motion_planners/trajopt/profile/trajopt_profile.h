/**
 * @file trajopt_profile.h
 * @brief
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/manipulator_info.h>
#include <tesseract_motion_planners/trajopt/visibility_control.h>

namespace tesseract_planning
{
class TESSERACT_MOTION_PLANNERS_TRAJOPT_PUBLIC TrajOptPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptPlanProfile>;

  virtual void apply(trajopt::ProblemConstructionInfo& pci,
                     const Eigen::Isometry3d& cartesian_waypoint,
                     const Instruction& parent_instruction,
                     const ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     int index) = 0;

  virtual void apply(trajopt::ProblemConstructionInfo& pci,
                     const Eigen::VectorXd& joint_waypoint,
                     const Instruction& parent_instruction,
                     const ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     int index) = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;
};

class TESSERACT_MOTION_PLANNERS_TRAJOPT_PUBLIC TrajOptCompositeProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptCompositeProfile>;

  virtual void apply(trajopt::ProblemConstructionInfo& pci,
                     int start_index,
                     int end_index,
                     const ManipulatorInfo& manip_info,
                     const std::vector<std::string>& active_links,
                     const std::vector<int>& fixed_indices) = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;
};

using TrajOptCompositeProfileMap = std::unordered_map<std::string, TrajOptCompositeProfile::Ptr>;
using TrajOptPlanProfileMap = std::unordered_map<std::string, TrajOptPlanProfile::Ptr>;
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H
