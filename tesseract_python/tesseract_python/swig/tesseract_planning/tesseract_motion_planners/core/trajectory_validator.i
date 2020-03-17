/**
 * @file trajectory_validator.i
 * @brief SWIG wrapper for Trajectory Validator Class
 *
 * @author Michael Ripperger
 * @date March 16, 2020
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

%{
#include <tesseract_motion_planners/core/trajectory_validator.h>
%}

namespace tesseract_motion_planners
{
enum class PostPlanCheckType : unsigned
{
  NONE = 0x0,
  SINGLE_TIMESTEP_COLLISION = 0x1,
  DISCRETE_CONTINUOUS_COLLISION = 0x2,
  CAST_CONTINUOUS_COLLISION = 0x4
};

class TrajectoryValidator
{
public:
  TrajectoryValidator(tesseract_collision::ContinuousContactManager::Ptr continuous_manager = nullptr,
                      tesseract_collision::DiscreteContactManager::Ptr discrete_manager = nullptr,
                      double longest_valid_segment_length = 0.01,
                      bool verbose = false);

  virtual bool trajectoryValid(const tesseract_common::TrajArray& trajectory,
                               const PostPlanCheckType& check_type,
                               const tesseract_environment::StateSolver& state_solver,
                               const std::vector<std::string>& joint_names);

protected:
  tesseract_collision::ContinuousContactManager::Ptr continuous_contact_manager_;
  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_;

  double longest_valid_segment_length_;
  bool verbose_;
};

} // namespace tesseract_motion_planners
