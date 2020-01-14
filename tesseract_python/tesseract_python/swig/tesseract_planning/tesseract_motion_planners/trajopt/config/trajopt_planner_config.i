/**
 * @file trajopt_planner_config.i
 * @brief SWIG interface file for tesseract_planning/trajopt/config/trajopt_planner_config.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
%}

%shared_ptr(tesseract_motion_planners::TrajOptPlannerConfig)

namespace tesseract_motion_planners
{
struct TrajOptPlannerConfig
{
  using Ptr = std::shared_ptr<TrajOptPlannerConfig>;

  explicit TrajOptPlannerConfig() = default;
  explicit TrajOptPlannerConfig(trajopt::TrajOptProb::Ptr problem);

  virtual ~TrajOptPlannerConfig() = default;

  virtual bool generate();

  sco::BasicTrustRegionSQPParameters params;

  //TODO: ?
  //std::vector<sco::Optimizer::Callback> callbacks;
  
  trajopt::TrajOptProb::Ptr prob;

  double longest_valid_segment_fraction = 0.01;

  double longest_valid_segment_length = 0.5;
};

}  // namespace tesseract_motion_planners
